% Start the  thing in the right mode
setmodeclient = rossvcclient('/mavros/set_mode');
setmodereq = rosmessage(setmodeclient);
setmodereq.CustomMode = 'OFFBOARD';
setmoderesp = call(setmodeclient,setmodereq);

% Arm the bot
armingclient = rossvcclient('/mavros/cmd/arming');
armingreq = rosmessage(armingclient);
armingreq.Value = 1;
armingresp = call(armingclient,armingreq);

% Publish one message to get it to the right position
[pub,msg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
msg.Pose.Position.X = 0;
msg.Pose.Position.Y = 0;
msg.Pose.Position.Z = 1.5;

eul = [0 0 0];
% Weird that this works because qZYX is [w, x, y, z]
% Furthermore, after launching, it requires the correct ordering.
qZYX = eul2quat(eul);
msg.Pose.Orientation.X = qZYX(1);
msg.Pose.Orientation.Y = qZYX(2);
msg.Pose.Orientation.Z = qZYX(3);
msg.Pose.Orientation.W = qZYX(4);
send(pub,msg); 