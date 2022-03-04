function dstatedt = Satellite(t,state)
%%genericState = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];
global m invI I

%%select states
x = state(1);
y = state(2);
z = state(3);
q0123 = state(7:10);
ptp = Quat2Eu(q0123)';
p = state(11);
q = state(12);
r = state(13);
pqr = state(11:13);

%%%Translational Kinematics
vel = state(4:6); %%Velocity

%%%Rotational Kinematics
%Derivative of Quaternions
PQRMAT = [0 -p -q -r;p 0 r -q;q -r 0 p;r q -p 0];
q0123dot = 0.5*PQRMAT*q0123;

%%%Gravity Model (Newton's law universal gravitation)
%%Load earth parameters
planet
r = state(1:3); %% r = [x;y;z]
rho = norm(r);
rhat = r/rho;
Fgrav = -(G*M*m/rho^2)*rhat;

%%%Translational Dynamics
F = Fgrav;
accel = F/m;

%%%Rotational Dynamics
%%%Total External Disturbance Moments
LMN = [0;0;0];
H = I*pqr;
pqrdot = invI*(LMN - cross(pqr,H));

%%%Return Derivatives State
dstatedt = [vel;accel;q0123dot;pqrdot];

%%%Discretization (check this)
%dt = 1; 
%dstatedt_dis = state + dstatedt*dt;
end