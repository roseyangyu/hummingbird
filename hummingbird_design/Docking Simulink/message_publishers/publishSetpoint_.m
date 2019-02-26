% Publishes a setpoint to the tailsitter
% pos = [x,y,z]
% quat = [w, x, y, z]
% both assumed in map frame
function published = publishSetpoint_(pos, yaw, pub, msg)
    msg.Pose.Position.X = pos(1);
    msg.Pose.Position.Y = pos(2);
    msg.Pose.Position.Z = pos(3);
    quat = eul2quat([yaw, 0, 0]);
    msg.Pose.Orientation.X = quat(2);
    msg.Pose.Orientation.Y = quat(3);
    msg.Pose.Orientation.Z = quat(4);
    msg.Pose.Orientation.W = quat(1);
    send(pub, msg);
    published = 1;
end