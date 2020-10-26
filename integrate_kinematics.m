function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot

%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot

%   new_state is the state after integration, also in the form [x;y;theta]
x0 = state(1);
y0 = state(2);
theta0 = state(3);

theta = theta0 + dt*ang_velocity;
%restrict theta to [0, 2*pi)
while theta >= 2*pi
    theta = theta - 2*pi;
end

if ang_velocity~=0
    x = x0 + (1/ang_velocity)*(sin(theta0+dt*ang_velocity) - sin(theta0))*lin_velocity;
    y = y0 + (1/ang_velocity)*(cos(theta0) - cos(theta0+dt*ang_velocity))*lin_velocity;
else
    x = x0 + dt*cos(theta0)*lin_velocity;
    y = y0 + dt*sin(theta0)*lin_velocity;
end

new_state = [x, y, theta];

end