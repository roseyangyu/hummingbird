#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <apriltags2_msgs/AprilTagDetectionPoseArray.h>
#include <string>
#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

void poseCallback(const nav_msgs::OdometryConstPtr& msg) {
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "base_link";
	
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
	transformStamped.transform.translation.y = msg->pose.pose.position.y;
	transformStamped.transform.translation.z = msg->pose.pose.position.z;
	transformStamped.transform.rotation = msg->pose.pose.orientation;

	br.sendTransform(transformStamped);
}

void cameraLocationCallback(const apriltags2_msgs::AprilTagDetectionPoseArrayConstPtr& msg) {
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped cameraOrientation;

	cameraOrientation.header.stamp = ros::Time::now();
	cameraOrientation.header.frame_id = "base_link";
	cameraOrientation.child_frame_id = "camera";

	cameraOrientation.transform.translation.x = 0;
	cameraOrientation.transform.translation.y = -0.006;
	cameraOrientation.transform.translation.z = 0.222;
	
	tf2::Matrix3x3 m(0,0,1,-1,0,0,0,-1,0);
	tf2::Quaternion q;
	m.getRotation(q);

	cameraOrientation.transform.rotation.x = q.x();
	cameraOrientation.transform.rotation.y = q.y();
	cameraOrientation.transform.rotation.z = q.z();
	cameraOrientation.transform.rotation.w = q.w();

	br.sendTransform(cameraOrientation);
}

void stars_realsense_callback(const geometry_msgs::TransformStampedConstPtr& msg) {
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped cameraOrientation;

	cameraOrientation.header.stamp = ros::Time::now();
	cameraOrientation.header.frame_id = "vicon/STARS_R200/STARS_R200";
	cameraOrientation.child_frame_id = "camera";

	cameraOrientation.transform.translation.x = -0.013;
	cameraOrientation.transform.translation.y = 0.008;
	cameraOrientation.transform.translation.z = 0.015;

	tf2::Matrix3x3 m(0,0,1,-1,0,0,0,-1,0);
	tf2::Quaternion q;
	m.getRotation(q);

	cameraOrientation.transform.rotation.x = q.x();
	cameraOrientation.transform.rotation.y = q.y();
	cameraOrientation.transform.rotation.z = q.z();
	cameraOrientation.transform.rotation.w = q.w();

	br.sendTransform(cameraOrientation);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_transformer");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("/mavros/global_position/local", 10,
			&poseCallback);
	ros::Subscriber sub2 = node.subscribe("/vicon/STARS_R200/STARS_R200", 10,
										 &stars_realsense_callback);
	ros::Subscriber sub3 = node.subscribe("/tag_detections", 10,
			&cameraLocationCallback);
	ros::spin();
	return 0;
}
