% these are used in the wrapper publishers
% we pre-allocate because they take a long time
[pub1,msg1] = rospublisher('/mavros/setpoint_raw/local','mavros_msgs/PositionTarget');
[pub2,msg2] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
