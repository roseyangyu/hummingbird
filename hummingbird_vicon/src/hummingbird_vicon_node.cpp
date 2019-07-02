#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

#include <string>

ros::Subscriber tf_sub;
ros::Publisher mavros_mocap_pub;
tf2_ros::Buffer tfBuffer;

geometry_msgs::TransformStamped COM_transform;
geometry_msgs::PoseStamped mocap_COM_pose;

std::string vicon_frame_id = "vicon/STARS_TS3/main";

int main(int argc, char **argv) {
    ros::init(argc, argv, "hummingbird_vicon_node");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle n;
    mavros_mocap_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1000);
    while (n.ok()) {
        try {
            tfBuffer.lookupTransform("world", vicon_frame_id, ros::Time::now(), ros::Duration(1)); // block for 1 second waiting for latest transform
            // if new transform available, look up world->COM and publih to /maros/mocap/pose
            COM_transform = tfBuffer.lookupTransform("world", "COM", ros::Time(0));
            mocap_COM_pose.pose.position.x = COM_transform.transform.translation.x;
            mocap_COM_pose.pose.position.y = COM_transform.transform.translation.y;
            mocap_COM_pose.pose.position.z = COM_transform.transform.translation.z;
            mocap_COM_pose.pose.orientation.x = COM_transform.transform.rotation.x;
            mocap_COM_pose.pose.orientation.y = COM_transform.transform.rotation.y;
            mocap_COM_pose.pose.orientation.z = COM_transform.transform.rotation.z;
            mocap_COM_pose.pose.orientation.w = COM_transform.transform.rotation.w;
            mavros_mocap_pub.publish(mocap_COM_pose);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        }
    }

    ros::spin();
    return 0;
}
