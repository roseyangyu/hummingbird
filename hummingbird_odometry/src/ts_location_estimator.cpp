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

#include <map>
#include <string>
#include <vector>

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
string meFrameId = "base_link";
string partnerFrameId = "partner";
SystemModel sys;
Kalman::ExtendedKalmanFilter<State> predictor;
Vector3f g(0, 0, -9.82); // gravity in Toronto, Canada

// frames to which a correction is applied
vector<string> tagFrames = {"bundle1", "tag3"};
const int NUM_TAGS = 2; 
Matrix3f correctionMatrices[NUM_TAGS] = {
    (Matrix3f() << 0,-1,0,0,0,1,-1,0,0).finished(), // bundle1
    (Matrix3f() << 0,-1,0,0,0,1,-1,0,0).finished(), // tag3
};
Vector3f tagOffsets[NUM_TAGS] = {
    (Vector3f() << -0.02, -0.163, 0.350).finished(), // bundle1
    (Vector3f() << -0.02, -0.163, 0.350).finished(), // tag3
};

// Elements of this array must be in tagFrames
// Measurements of these tags is used to update the state
vector<string> stateUpdateIds = {"bundle1"};


ros::Time previous;

geometry_msgs::TransformStamped currentPartnerEstimate;
State x;
Control u;

bool start = false;

// Updates the estimated partner transformation
// Returns true if the estimate was updated
// Tag number is [1, TAG_NUMBER]
bool updatePartnerPosition(PositionMeasurement positionMeasurement,
                             OrientationMeasurement orientationMeasurement,
                             SystemModel sys,
                             Kalman::ExtendedKalmanFilter<State>& predictor,
                             PositionModel pm,
                             OrientationModel om) {
    predictor.update(pm, positionMeasurement);
    x = predictor.update(om, orientationMeasurement);
    return true; // Estimate position of partner as the position of the 3rd tag for now.
}

void packState() {
    currentPartnerEstimate.transform.translation.x = x(0);
    currentPartnerEstimate.transform.translation.y = x(1);
    currentPartnerEstimate.transform.translation.z = x(2);   
    // Why inverse?
    // the state is defined as the rotation that transforms the partner frame (modelled as inertial)
    // to the base_link frame. 
    Quaternionf q = eulerToQuaternion(x(6), x(7), x(8)).inverse();
    currentPartnerEstimate.transform.rotation.x = q.x();
    currentPartnerEstimate.transform.rotation.y = q.y();
    currentPartnerEstimate.transform.rotation.z = q.z();
    currentPartnerEstimate.transform.rotation.w = q.w();
}

void predictPartnerPosition(Kalman::ExtendedKalmanFilter<State>& predictor, SystemModel sys, Control u, float dt) {
    x = predictor.predict(sys, u, dt);
}

ros::Duration computeDt()
{
    ros::Time now = ros::Time::now();
    ros::Duration dt = now - previous;
    previous = now;
    return dt;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ros::Duration dt = computeDt();
    predictPartnerPosition(predictor, sys, u, dt.toSec());

    u(0) = msg->linear_acceleration.x;
    u(1) = msg->linear_acceleration.y;
    u(2) = msg->linear_acceleration.z;
    u(3) = msg->angular_velocity.x;
    u(4) = msg->angular_velocity.y;
    u(5) = msg->angular_velocity.z;

    Quaternionf q = eulerToQuaternion(x(6), x(7), x(8)).inverse();
    // we assume that gravity is pointed downward in partner's frame (e.g. partner is level)
    u.segment(0,3) = u.segment(0,3) + q*g;
}

/*
 * This node attempts to determine the location of the partner tailsitter relative to itself.
 * It does this by looking up the transformations from itself to the apriltags, adding a correction to orientate
 * correctly and adjust offsets, and calculates an estimate of the partner's location
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "apriltag_destination");

    ros::NodeHandle node;

    currentPartnerEstimate.header.frame_id = meFrameId;
    currentPartnerEstimate.child_frame_id = partnerFrameId;
   
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    previous = ros::Time::now();

    ros::Subscriber imu_sub = node.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1000, imuCallback);

    ros::Rate rate(60.0);

    PositionModel pm;
    OrientationModel om;
    PositionMeasurement pMeasurement;
    OrientationMeasurement oMeasurement;

    x.setZero();
    u.setZero();
    predictor.init(x);

    geometry_msgs::TransformStamped tagTransforms[NUM_TAGS];
    float lastUsedTimes[NUM_TAGS] = {0};

    string tagFrameId;
    // TODO: change tag lookup to a subscription?
    while (node.ok()){
        geometry_msgs::TransformStamped tagTransforms[NUM_TAGS];
        tf2::Quaternion qCorrect;
        ros::Duration dt = computeDt();
        predictPartnerPosition(predictor, sys, u, dt.toSec());
        for (int i = 0; i < tagFrames.size(); ++i) {
            tagFrameId = tagFrames[i];
            // Lookup transform from base_link to ith tag
            try {
                tagTransforms[i] = tfBuffer.lookupTransform(meFrameId, tagFrameId, ros::Time(0));
            } catch (tf2::TransformException & ex){
                //ROS_WARN("%s",ex.what());
                continue;
            }
            float tagTime = tagTransforms[i].header.stamp.sec + tagTransforms[i].header.stamp.nsec/1000000000.0;
            // Check if we've already used this tag via timestamp comparison
            if (tagTime == lastUsedTimes[i]) {
                continue;
            } else {
                lastUsedTimes[i] = tagTime;
            }

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
            tagTransforms[i].header.stamp = ros::Time::now();
            tagTransforms[i].child_frame_id = tagFrameId + "_corrected";
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
            if (std::find(stateUpdateIds.begin(), stateUpdateIds.end(), tagFrameId) != stateUpdateIds.end() &&
                updatePartnerPosition(pMeasurement, oMeasurement, sys, predictor, pm, om)) {
                start = true; // first estimate successful, begin outputting
            }
        }
        // TODO: Need to change message type to output covariance as well.
        // PoseWithCovariance
        if (start) {
            currentPartnerEstimate.header.stamp = ros::Time::now();
            packState();
            br.sendTransform(currentPartnerEstimate);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
