% Initialize section
% Puts TS in starting location
numSecsPerRev = 20;
omega = 2*pi/numSecsPerRev;
numRevs = 1;
zOffset = 1.74;
radius = 0.3;
tend = posixtime(datetime('now')) + 2*pi*numRevs/omega;
yaw = 0;

% Put TS at start of circle
for i=1:100
    publishSetpoint([radius;0;zOffset],0);
    pause(0.1)
end

%%
numSecsPerRev = 20;
omega = 2*pi/numSecsPerRev;
numRevs = 1;
zOffset = 1.74;
radius = 0.3;
yaw = 0;
tic
t0 = posixtime(datetime('now'));
tend = t0 + 2*pi*numRevs/omega;
t = 0;
display('starting circle')
while t0+t < tend
    t = posixtime(datetime('now'))-t0;
    pos = [radius*cos(omega*t);...
           radius*sin(omega*t);...
           zOffset];
    vel = [-1*radius*omega*sin(omega*t);...
           radius*omega*cos(omega*t);...
           0];
    publishRawSetpoint(pos, vel, yaw);
    publishRawSetpoint(pos, vel, yaw);
    publishRawSetpoint(pos, vel, yaw);
    pause(0.1);
end
toc