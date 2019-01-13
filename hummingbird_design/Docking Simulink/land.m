% Publish one message to get it to the right position
[pub,msg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
msg.Pose.Position.X = 2;
msg.Pose.Position.Y = 0;
msg.Pose.Position.Z = 0;
send(pub,msg); 