#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

Vector3d comTranslation;
Quaterniond comRotation;

Vector3d tsTranslation;
Quaterniond tsRotation;

ros::Subscriber tf_sub;
ros::Publisher mavros_mocap_pub;
ros::Subscriber vicon_sub;
tf2_ros::Buffer tfBuffer;
geometry_msgs::PoseStamped comPose;

std::string vicon_frame = "vicon/STARS_TS3/STARS_TS3";

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    tsTranslation(0) = msg->transform.translation.x;
    tsTranslation(1) = msg->transform.translation.y;
    tsTranslation(2) = msg->transform.translation.z;
    tsRotation.x() = msg->transform.rotation.x;
    tsRotation.y() = msg->transform.rotation.y;
    tsRotation.z() = msg->transform.rotation.z;
    tsRotation.w() = msg->transform.rotation.w;

    tsTranslation = tsTranslation + tsRotation*comTranslation;
    tsRotation = tsRotation*comRotation;

    comPose.header.stamp = msg->header.stamp;
    comPose.pose.position.x = tsTranslation(0); 
    comPose.pose.position.y = tsTranslation(1);
    comPose.pose.position.z = tsTranslation(2);
    comPose.pose.orientation.x = tsRotation.x();
    comPose.pose.orientation.y = tsRotation.y();
    comPose.pose.orientation.z = tsRotation.z();
    comPose.pose.orientation.w = tsRotation.w();
    mavros_mocap_pub.publish(comPose);
}

// Why not just do a tf2 lookup?
// It is too slow.
int main(int argc, char **argv) {
    ros::init(argc, argv, "hummingbird_vicon_node");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle n;
    mavros_mocap_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1000);
    vicon_sub = n.subscribe("/" + vicon_frame, 1000, viconCallback);
    try {
        geometry_msgs::TransformStamped tsToCom;
        tsToCom = tfBuffer.lookupTransform(vicon_frame, "COM", ros::Time::now(), ros::Duration(10)); // block for 1 second waiting for latest transform
        comTranslation(0) = tsToCom.transform.translation.x;
        comTranslation(1) = tsToCom.transform.translation.y;
        comTranslation(2) = tsToCom.transform.translation.z;
        comRotation.x() = tsToCom.transform.rotation.x;
        comRotation.y() = tsToCom.transform.rotation.y;
        comRotation.z() = tsToCom.transform.rotation.z;
        comRotation.w() = tsToCom.transform.rotation.w;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
    ros::spin();
    return 0;
}
