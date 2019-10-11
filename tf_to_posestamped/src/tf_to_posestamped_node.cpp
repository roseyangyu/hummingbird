#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>


ros::Publisher pose_publisher;
tf2_ros::Buffer tfBuffer;

std::string parent_frame_id;
std::string child_frame_id;
std::string pose_output_topic;

using namespace Eigen;

Vector3d poseTranslation;
Quaterniond poseRotation;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_to_posestamped_node");
  ros::NodeHandle n_pr("~");
  ros::NodeHandle n;
  n_pr.param<std::string>("pose_output_topic", pose_output_topic, "pose_output_topic");
  n_pr.param<std::string>("parent_frame_id", parent_frame_id, "parent_frame");
  n_pr.param<std::string>("child_frame_id", child_frame_id, "child_frame");
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_output_topic, 1000);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transf;
  geometry_msgs::PoseStamped pose;
  while(n.ok()) {
    try {
      geometry_msgs::TransformStamped transf;
      transf = tfBuffer.lookupTransform(parent_frame_id, child_frame_id, ros::Time::now(), ros::Duration(1)); // block for 1 second waiting for latest transform
      pose.header = transf.header;
      poseTranslation(0) = transf.transform.translation.x;
      poseTranslation(1) = transf.transform.translation.y;
      poseTranslation(2) = transf.transform.translation.z;
      poseRotation.x() = transf.transform.rotation.x;
      poseRotation.y() = transf.transform.rotation.y;
      poseRotation.z() = transf.transform.rotation.z;
      poseRotation.w() = transf.transform.rotation.w;

      pose.pose.position.x = poseTranslation(0);
      pose.pose.position.y = poseTranslation(1);
      pose.pose.position.z = poseTranslation(2);
      pose.pose.orientation.x = poseRotation.x();
      pose.pose.orientation.y = poseRotation.y();
      pose.pose.orientation.z = poseRotation.z();
      pose.pose.orientation.w = poseRotation.w();
      pose_publisher.publish(pose);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
  }

  return 0;
}