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

int main(int argc, char** argv){
    ros::init(argc, argv, "apriltag_destination");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    ros::Rate rate(60.0);

    while (node.ok()){

        geometry_msgs::PoseArray poseArray;

        int totalTags = 0;

        vector<geometry_msgs::PoseStamped> possiblePoses;
        vector<geometry_msgs::TransformStamped> possibleTransforms;

        for (int i = 1; i < 7; ++i) {
            try{
                geometry_msgs::TransformStamped world_apriltag;
                world_apriltag = tfBuffer.lookupTransform("world", "tag" + to_string(i), ros::Time(0));

                geometry_msgs::TransformStamped ts_transform;
                tf2::Quaternion q;
                if(i == 1) {

                    ts_transform.transform.translation.x = 0.14;
                    ts_transform.transform.translation.y = -0.157; // -0.357
                    ts_transform.transform.translation.z = -0.02;

                    tf2::Matrix3x3 m(0,-1,0,0,0,1,-1,0,0);
                    m.getRotation(q);

                }
                else if (i == 2) {
                    ts_transform.transform.translation.x = -0.14;
                    ts_transform.transform.translation.y = -0.157; // -0.357
                    ts_transform.transform.translation.z = -0.02;

                    tf2::Matrix3x3 m(0,-1,0,0,0,1,-1,0,0);
                    m.getRotation(q);
                }
                else if (i == 3) {
                    ts_transform.transform.translation.x = -0.14;
                    ts_transform.transform.translation.y = -0.157; // -0.357
                    ts_transform.transform.translation.z = -0.02;

                    tf2::Matrix3x3 m(0,1,0,0,0,1,1,0,0);
                    m.getRotation(q);
                }
                else if (i == 4) {
                    ts_transform.transform.translation.x = 0.14;
                    ts_transform.transform.translation.y = -0.157; // -0.357
                    ts_transform.transform.translation.z = -0.02;

                    tf2::Matrix3x3 m(0,1,0,0,0,1,1,0,0);
                    m.getRotation(q);
                }
                else if (i == 5) {
                    ts_transform.transform.translation.x = -0.054;
                    ts_transform.transform.translation.y = -0.182; // -0.382
                    ts_transform.transform.translation.z = -0.02;

                    tf2::Matrix3x3 m(0,1,0,0,0,1,1,0,0);
                    m.getRotation(q);
                }
                else if (i == 6) {
                    ts_transform.transform.translation.x = 0.054;
                    ts_transform.transform.translation.y = -0.182; // -0.382
                    ts_transform.transform.translation.z = -0.02;

                    tf2::Matrix3x3 m(0,1,0,0,0,1,1,0,0);
                    m.getRotation(q);
                }

                ts_transform.header.stamp = ros::Time::now();
                ts_transform.header.frame_id = "tag" + to_string(i);
                ts_transform.child_frame_id = "t" + to_string(i);
                ts_transform.transform.rotation.x = q.x();
                ts_transform.transform.rotation.y = q.y();
                ts_transform.transform.rotation.z = q.z();
                ts_transform.transform.rotation.w = q.w();

                br.sendTransform(ts_transform);

                // Calibration
                geometry_msgs::Transform tagCalibration = load_calibration(node, "tag" + to_string(i));

                geometry_msgs::TransformStamped tagCalibrationStamped;
                tagCalibrationStamped.transform = tagCalibration;
                tagCalibrationStamped.header = ts_transform.header;
                tagCalibrationStamped.header.frame_id = "t" + to_string(i);
                tagCalibrationStamped.child_frame_id = "ts" + to_string(i);
                br.sendTransform(tagCalibrationStamped);

            }
            catch (tf2::TransformException & ex){
                continue;
            }
        }

        rate.sleep();
    }
    return 0;
};
