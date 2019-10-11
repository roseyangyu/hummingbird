function published = publishSetpoint(pos, yaw)
    pub2 = evalin('base', 'pub2');
    msg2 = evalin('base', 'msg2');
    published = publishSetpoint_(pos, yaw, pub2, msg2);
end