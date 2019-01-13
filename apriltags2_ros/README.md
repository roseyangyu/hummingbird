# apriltags2_ros

This is a accelerated version of Apriltags2 on ROS. It takes frames in between the images and uses lukas kanade optical flow to predict the location of the apriltag. Using this method, you can remove the latency caused by AprilTag detection and the speed of the apriltag is limited by the image frame rate and the optical flow rate instead.
