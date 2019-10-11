function published = publishRawSetpoint(pos, vel, yaw)
    pub1 = evalin('base', 'pub1');
    msg1 = evalin('base', 'msg1');
    published = publishRawSetpoint_(pos, vel, yaw, pub1, msg1);
end
