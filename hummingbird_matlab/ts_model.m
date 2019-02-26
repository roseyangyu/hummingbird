%% Full Equations of model
syms Ixx Iyy Izz
syms kt km kl kd
syms w1 w2
syms d1 d2
syms l m
syms ax ay az
syms alphax alphay alphaz
syms g
syms yaw pitch roll
syms yaw_dot pitch_dot roll_dot
syms t
syms u v w

I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
a = [ax;ay;az];
alpha = [alphax; alphay; alphaz];

R = rotz(yaw)*roty(pitch)*rotx(roll);
g_body = R'*[0;0;g];

Force = [kl*(d1*w1^2 + d2*w2^2);...
          0;...
          kd*(d1^2*w1^2+d2^2*w2^2) - kt*(w1^2+w2^2)] + m*g_body;
Torque = [kt*l*(w1^2-w2^2);...
          km*(d1*w1^2+d2*w2^2);...
          km*(w1^2-w2^2)];
      
equations = [Force == m*a; Torque == I*alpha;]
unknowns = [w1;w2;d1;d2];
%% Here we simplify by assuming no drag, no gravity
syms Ixx Iyy Izz
syms kt km kl kd
syms w1 w2
syms d1 d2
syms l m
syms ax ay az
syms alphax alphay alphaz
syms g
g = 0;
syms yaw pitch roll
syms yaw_dot pitch_dot roll_dot
syms t
syms u v w

I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
a = [ax;ay;az];
alpha = [alphax; alphay; alphaz];

R = rotz(yaw)*roty(pitch)*rotx(roll);
g_body = R'*[0;0;g];

Force = [kl*(d1*w1^2 + d2*w2^2);...
          0;...
          kd*(d1^2*w1^2+d2^2*w2^2) - kt*(w1^2+w2^2)] + m*g_body;
Torque = [kt*l*(w1^2-w2^2);...
          km*(d1*w1^2+d2*w2^2);...
          km*(w1^2-w2^2)];
      
equations = [Force == m*a; Torque == I*alpha;];
equations = equations(3:end)
unknowns = [w1;w2;d1;d2];
%% 
% Used this section to compute derivative of rpy wrt time as a function
% of the body angular rates (given by the gyro)
syms yaw pitch roll
syms yaw_dot pitch_dot roll_dot
syms u v w
body_rate_equations = [u;v;w] == rotx(roll)'*roty(pitch)'*[0;0;yaw_dot] + rotx(roll)'*[0;pitch_dot;0] + [roll_dot;0;0]
rpy = [roll_dot;pitch_dot;yaw_dot]
inverse = solve(body_rate_equations, rpy)
pretty(simplify(diff(inverse.roll_dot,pitch)))