#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/Types.hpp>
#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "OrientationMeasurementModel.hpp"

#include <sensor_msgs/Imu.h>

using namespace std;
using namespace KalmanExamples;
using namespace Eigen;

/* Typedefs */
typedef float T;
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;
typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::OrientationMeasurement<T> OrientationMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;
typedef Robot1::OrientationMeasurementModel<T> OrientationModel;

/* Global constants */
string me_frame_id = "base_link";
string partner_frame_id = "partner";
const int NUM_TAGS = 6; 

Matrix3f correctionMatrices[NUM_TAGS] = {
    (Matrix3f() << 0,1,0,0,0,1,1,0,0).finished(),
    (Matrix3f() << 0,1,0,0,0,1,1,0,0).finished(),
    (Matrix3f() << 0,-1,0,0,0,1,-1,0,0).finished(),
    (Matrix3f() << 0,-1,0,0,0,1,-1,0,0).finished(),
    (Matrix3f() << 0,-1,0,0,0,1,-1,0,0).finished(),
    (Matrix3f() << 0,-1,0,0,0,1,-1,0,0).finished(), 
};
Vector3f tagOffsets[NUM_TAGS] = {
    (Vector3f() << 0.02, -0.14, 0.357).finished(),
    (Vector3f() << 0.02, 0.14, 0.357).finished(),
    (Vector3f() << -0.02, -0.14, 0.357).finished(),
    (Vector3f() << -0.02, 0.14, 0.357).finished(),
    (Vector3f() << -0.02, -0.054, 0.382).finished(),
    (Vector3f() << -0.02, 0.054, 0.382).finished(), 
};

geometry_msgs::TransformStamped currentPartnerEstimate;
State x;
Control u;

geometry_msgs::Transform load_calibration(ros::NodeHandle& nh, string transform_name) {
    double x, y, z, rx, ry, rz, rw;
    nh.param<double>("odometry/" + transform_name + "/calib/translation/x", x, 0.0);
    nh.param<double>("odometry/" + transform_name + "/calib/translation/y", y, 0.0);
    nh.param<double>("odometry/" + transform_name + "/calib/translation/z", z, 0.0);
    nh.param<double>("odometry/" + transform_name + "/calib/rotation/x", rx, 0.0);
    nh.param<double>("odometry/" + transform_name + "/calib/rotation/y", ry, 0.0);
    nh.param<double>("odometry/" + transform_name + "/calib/rotation/z", rz, 0.0);
    nh.param<double>("odometry/" + transform_name + "/calib/rotation/w", rw, 1.0);

    geometry_msgs::Transform transform;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.translation.z = z;
    transform.rotation.x = rx;
    transform.rotation.y = ry;
    transform.rotation.z = rz;
    transform.rotation.w = rw;

    cout << transform_name << " " << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << " " << rw << " " << endl;

    return transform;
}

// Updates the estimated partner transformation
// Returns true if the estimate was updated
// Tag number is [1, TAG_NUMBER]
bool estimatePartnerPosition(PositionMeasurement positionMeasurement,
                             OrientationMeasurement orientationMeasurement,
                             int tagNumber, 
                             SystemModel sys,
                             Kalman::ExtendedKalmanFilter<State>& predictor,
                             PositionModel pm,
                             OrientationModel om,
                             float dt) {
    currentPartnerEstimate.header.stamp = ros::Time::now();
    currentPartnerEstimate.header.frame_id = me_frame_id;
    currentPartnerEstimate.child_frame_id = partner_frame_id;
    if (tagNumber == 3) {
        predictor.predict(sys, dt);
        predictor.update(pm, positionMeasurement);
        auto x_ekf = predictor.update(om, orientationMeasurement);
        currentPartnerEstimate.transform.translation.x = x_ekf(0);
        currentPartnerEstimate.transform.translation.y = x_ekf(1);
        currentPartnerEstimate.transform.translation.z = x_ekf(2);
        // Why inverse?
        // the state is defined as the rotation from partner_frame (modelled as inertial) to base_link frame
        Quaternionf q = eulerToQuaternion(x_ekf(6), x_ekf(7), x_ekf(8)).inverse();
        currentPartnerEstimate.transform.rotation.x = q.x();
        currentPartnerEstimate.transform.rotation.y = q.y();
        currentPartnerEstimate.transform.rotation.z = q.z();
        currentPartnerEstimate.transform.rotation.w = q.w();
        return true; // Estimate position of partner as the position of the 3rd tag for now.
    }
    return false;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    u(0) = msg->linear_acceleration.x;
    u(1) = msg->linear_acceleration.y;
    u(2) = msg->linear_acceleration.z;
    u(3) = msg->angular_velocity.x;
    u(4) = msg->angular_velocity.y;
    u(5) = msg->angular_velocity.z;
}

/*
 * This node attempts to determine the location of the partner tailsitter relative to itself.
 * It does this by looking up the transformations from itself to the apriltags, adding a correction to orientate
 * correctly and adjust offsets, and calculates an estimate of the partner's location
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "apriltag_destination");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    ros::Subscriber imu_sub = node.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, imuCallback);

    ros::Rate rate(60.0);

    PositionModel pm;
    OrientationModel om;
    PositionMeasurement pMeasurement;
    OrientationMeasurement oMeasurement;
    x.setZero();
    u.setZero();

    SystemModel sys;

    Kalman::ExtendedKalmanFilter<State> predictor;
    predictor.init(x);

    ros::Time previous = ros::Time::now();

    while (node.ok()){
        geometry_msgs::TransformStamped tagTransforms[NUM_TAGS];
        tf2::Quaternion qCorrect;
        ros::Time now = ros::Time::now();
        ros::Duration dt = now - previous;
        previous = now;
        for (int i = 0; i < NUM_TAGS; ++i) {
            string tag_frame_id = "tag" + to_string(i+1);
            // Lookup transform from base_link to ith tag
            try {
                tagTransforms[i] = tfBuffer.lookupTransform(me_frame_id, tag_frame_id, ros::Time(0));
            } catch (tf2::TransformException & ex){
                //ROS_WARN("%s",ex.what());
                continue;
            }
            // Unpack the data into eigen variables from geometry_msgs.
            // TODO: maybe put this in an external function?
            Vector3f origin(tagTransforms[i].transform.translation.x,
                            tagTransforms[i].transform.translation.y,
                            tagTransforms[i].transform.translation.z);
            Quaternionf qTag(tagTransforms[i].transform.rotation.w, 
                             tagTransforms[i].transform.rotation.x,
                             tagTransforms[i].transform.rotation.y,
                             tagTransforms[i].transform.rotation.z);

            Quaternionf qCorrect(correctionMatrices[i]);
            qTag = qTag*qCorrect;

            // Correct the origin
            origin = origin - qTag*tagOffsets[i];

            // Repack and broadcast as corrected tag
            tagTransforms[i].header.stamp = now;
            tagTransforms[i].child_frame_id = tag_frame_id + "_corrected";
            tagTransforms[i].transform.rotation.x = qTag.x();
            tagTransforms[i].transform.rotation.y = qTag.y();
            tagTransforms[i].transform.rotation.z = qTag.z();
            tagTransforms[i].transform.rotation.w = qTag.w();
            tagTransforms[i].transform.translation.x = origin(0);
            tagTransforms[i].transform.translation.y = origin(1);
            tagTransforms[i].transform.translation.z = origin(2);
            br.sendTransform(tagTransforms[i]);

            // Estimate based on measurements of tag_corrected
            pMeasurement = origin;
            oMeasurement = quaterniontoEulerAngle(qTag.inverse()); // inverse because state is rpy from partner to base_link
            if (estimatePartnerPosition(pMeasurement, oMeasurement, i+1, sys, predictor, pm, om, dt.toSec())) {
                br.sendTransform(currentPartnerEstimate);
            }
        }
        rate.sleep();
    }
    return 0;
};
