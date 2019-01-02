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

using namespace std;

geometry_msgs::Pose transformToPose(geometry_msgs::Transform & t) {
	geometry_msgs::Pose p;
	p.position.x = t.translation.x;
	p.position.y = t.translation.y;
	p.position.z = t.translation.z;
	p.orientation.x = t.rotation.x;
	p.orientation.y = t.rotation.y;
	p.orientation.z = t.rotation.z;
	p.orientation.w = t.rotation.w;
	return p;
}

geometry_msgs::PoseStamped transformToPoseStamped(geometry_msgs::Transform & t) {
	geometry_msgs::Pose p;
	p.position.x = t.translation.x;
	p.position.y = t.translation.y;
	p.position.z = t.translation.z;
	p.orientation.x = t.rotation.x;
	p.orientation.y = t.rotation.y;
	p.orientation.z = t.rotation.z;
	p.orientation.w = t.rotation.w;
	geometry_msgs::PoseStamped ps;
	ps.pose = p;
	return ps;
}

const int NUM_TAGS = 6;
ros::Publisher tag_publisher;

/*
 * This node publishes the location of the partner tailsitter relative to itself on /ts_location_estimates
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "tag_publisher");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);


  tag_publisher = node.advertise<geometry_msgs::PoseArray>("tag_array", 10);

  ros::Rate rate(20.0);

  while (node.ok()){
    
  	geometry_msgs::PoseArray pose_array;
    for (int i = 0; i < NUM_TAGS; i++) {
      geometry_msgs::Pose pose;
      try{
        geometry_msgs::TransformStamped baselinkToPartnerTransform = 
          tfBuffer.lookupTransform("base_link", "tag" + to_string(i+1) +"_corrected", ros::Time(0));
        pose = transformToPose(baselinkToPartnerTransform.transform);
      }
      catch (tf2::TransformException & ex){
        continue;
      }
      pose_array.poses.push_back(pose);
    }
    tag_publisher.publish(pose_array);
    rate.sleep();
  }
  return 0;
};