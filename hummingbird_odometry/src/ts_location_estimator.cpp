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
#include <tf/LinearMath/Matrix3x3.h>

using namespace std;

string me_frame_id = "base_link";
string partner_frame_id = "partner";
const int NUM_TAGS = 6; 
tf2::Matrix3x3 correctionMatricies[NUM_TAGS] = {
    tf2::Matrix3x3(0,1,0,0,0,1,1,0,0),
    tf2::Matrix3x3(0,1,0,0,0,1,1,0,0),
    tf2::Matrix3x3(0,-1,0,0,0,1,-1,0,0),
    tf2::Matrix3x3(0,-1,0,0,0,1,-1,0,0),
    tf2::Matrix3x3(0,-1,0,0,0,1,-1,0,0),
    tf2::Matrix3x3(0,-1,0,0,0,1,-1,0,0),
};
tf2::Vector3 tagOffsets[NUM_TAGS] = {
    tf2::Vector3(0.02, -0.14, 0.357), // tag0 pose: 0.02 -0.14 0.357
    tf2::Vector3(0.02, 0.14, 0.357), // 0.02 0.14 0.357
    tf2::Vector3(-0.02, -0.14, 0.357), // -0.02 -0.14 0.357
    tf2::Vector3(-0.02, 0.14, 0.357), // -0.02 0.14 0.357
    tf2::Vector3(-0.02, -0.054, 0.382), // -0.02 -0.054 0.382
    tf2::Vector3(-0.02, 0.054, 0.382), //-0.02 0.054 0.382
};

geometry_msgs::TransformStamped currentPartnerEstimate;
bool estimateValid = false; 

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
bool estimatePartnerPosition(geometry_msgs::TransformStamped tagTransform,
                             int tagNumber) {
    currentPartnerEstimate.header.stamp = ros::Time::now();
    currentPartnerEstimate.header.frame_id = me_frame_id;
    currentPartnerEstimate.child_frame_id = partner_frame_id;
    if (tagNumber == 3) {
        currentPartnerEstimate.transform = tagTransform.transform;
        return true; // Estimate position of partner as the position of the 3rd tag for now.
    }
    return false;
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

    ros::Rate rate(60.0);

    while (node.ok()){
        geometry_msgs::TransformStamped tagTransforms[NUM_TAGS];
        tf2::Quaternion qCorrect;

        for (int i = 0; i < NUM_TAGS; ++i) {
            string tag_frame_id = "tag" + to_string(i+1);
            // Lookup transform from base_link to ith tag
            try {
                tagTransforms[i] = tfBuffer.lookupTransform(me_frame_id, tag_frame_id, ros::Time(0));
            } catch (tf2::TransformException & ex){
                //ROS_WARN("%s",ex.what());
                continue;
            }

            // Unpack the data into tf2 variables from geometry_msgs.
            tf2::Vector3 origin(tagTransforms[i].transform.translation.x,
                                tagTransforms[i].transform.translation.y,
                                tagTransforms[i].transform.translation.z);
            tf2::Quaternion qTag(tagTransforms[i].transform.rotation.x, 
                                 tagTransforms[i].transform.rotation.y,
                                 tagTransforms[i].transform.rotation.z,
                                 tagTransforms[i].transform.rotation.w);

            // Find the orientation of the tag such that x faces towards us
            // TODO: change this to initialize quaternions from the start instead of computing again and again
            correctionMatricies[i].getRotation(qCorrect);
            qTag = qTag*qCorrect;
            tagTransforms[i].transform.rotation.x = qTag.x();
            tagTransforms[i].transform.rotation.y = qTag.y();
            tagTransforms[i].transform.rotation.z = qTag.z();
            tagTransforms[i].transform.rotation.w = qTag.w();

            // Account for the fact that the tag is not at the origin
            //R.setRotation(qTag); // TODO: How to do direct mult of quat with vector?
            //origin = origin - R*tagOffsets[i];
            origin = origin - tagOffsets[i].rotate(qTag.getAxis(), qTag.getAngle());
            tagTransforms[i].transform.translation.x = origin.m_floats[0];
            tagTransforms[i].transform.translation.y = origin.m_floats[1];
            tagTransforms[i].transform.translation.z = origin.m_floats[2];

            // Set header and publish
            tagTransforms[i].header.stamp = ros::Time::now();
            tagTransforms[i].child_frame_id = tag_frame_id + "_corrected";
            br.sendTransform(tagTransforms[i]);
            if (estimatePartnerPosition(tagTransforms[i], i+1)) {
                br.sendTransform(currentPartnerEstimate);
            }
        }
        rate.sleep();
    }
    return 0;
};
