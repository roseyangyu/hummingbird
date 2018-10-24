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

string me_frame_id = "base_link";
string partner_frame_id = "partner";
const int NUM_TAGS = 6; 
tf2::Matrix3x3 correctionMatricies[NUM_TAGS] = {
    tf2::Matrix3x3(0,-1,0,0,0,1,-1,0,0),
    tf2::Matrix3x3(0,-1,0,0,0,1,-1,0,0),
    tf2::Matrix3x3(0,1,0,0,0,1,1,0,0),
    tf2::Matrix3x3(0,1,0,0,0,1,1,0,0),
    tf2::Matrix3x3(0,1,0,0,0,1,1,0,0),
    tf2::Matrix3x3(0,1,0,0,0,1,1,0,0),
};
double tagOffsets[NUM_TAGS][3] = {
    {0.14, -0.157, -0.02},
    {-0.14, -0.157, -0.02},
    {-0.14, -0.157, -0.02},
    {0.14, -0.157, -0.02},
    {-0.054, -0.182, -0.02},
    {0.054, -0.182, -0.02}
};

geometry_msgs::TransformStamped currentPartnerEstimate;
bool estimateValid = false; 

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
            string from = me_frame_id;
            string to = "tag" + to_string(i+1);
            // Lookup transform from base_link to ith tag
            try {
                tagTransforms[i] = tfBuffer.lookupTransform(from, to, ros::Time(0));
            } catch (tf2::TransformException & ex){
                ROS_WARN("%s",ex.what());
                continue;
            }
            tf2::Quaternion qTag(tagTransforms[i].transform.rotation.x, 
                                 tagTransforms[i].transform.rotation.y,
                                 tagTransforms[i].transform.rotation.z,
                                 tagTransforms[i].transform.rotation.w);
            correctionMatricies[i].getRotation(qCorrect);
            qTag = qTag*qCorrect;
            tagTransforms[i].transform.rotation.x = qTag.x();
            tagTransforms[i].transform.rotation.y = qTag.y();
            tagTransforms[i].transform.rotation.z = qTag.z();
            tagTransforms[i].transform.rotation.w = qTag.w();
            // Set header and publish
            tagTransforms[i].header.stamp = ros::Time::now();
            tagTransforms[i].header.frame_id = from;
            tagTransforms[i].child_frame_id = to + "_corrected";
            br.sendTransform(tagTransforms[i]);
            if (estimatePartnerPosition(tagTransforms[i], i+1)) {
                br.sendTransform(currentPartnerEstimate);
            }
        }
        rate.sleep();
    }
    return 0;
};
