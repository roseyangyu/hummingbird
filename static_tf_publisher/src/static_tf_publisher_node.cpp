#include <string>
#include <iostream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <static_tf_publisher.hpp>

void publish_transform(std::string frame_id, 
					   std::string child_frame_id,
					   geometry_msgs::Quaternion rotation,
					   geometry_msgs::Vector3 translation) {
	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped transformation;
	transformation.header.stamp = ros::Time::now();
	transformation.header.frame_id = frame_id;
	transformation.child_frame_id = child_frame_id;
	transformation.transform.rotation = rotation;
	transformation.transform.translation = translation;
	br.sendTransform(transformation);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "static_tf_publisher");
	ros::NodeHandle node;

	std::string transformations_file;
	node.getParam("transformations_file", transformations_file);

	std::vector<Transformation> transformations = parse_transformations_yaml(transformations_file);
	for (int i = 0; i < transformations.size(); i++) {
		printf("Publishing static transform %s->%s\n", transformations[i].frame_id.c_str(), transformations[i].child_frame_id.c_str());
		printf("Rotation (x,y,z,w): %f, %f, %f, %f\n", 
				transformations[i].rotation.x,
				transformations[i].rotation.y,
				transformations[i].rotation.z,
				transformations[i].rotation.w);
		printf("Translation (x,y,z): %f, %f, %f\n",
				transformations[i].translation.x,
				transformations[i].translation.y,
				transformations[i].translation.z);
		publish_transform(transformations[i].frame_id,
						  transformations[i].child_frame_id,
						  transformations[i].rotation,
						  transformations[i].translation);
	}
    ros::spin();

	return 0;
}

