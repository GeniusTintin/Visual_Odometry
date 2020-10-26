%% initial conditions
close all
clear 
clc

addpath("../simulator/")
addpath("./Dataset/")
% load data from the dataset
load("myDataset_square.mat")
load("landmarks_square.mat") % used for RMSE calculation

% ground truth trajectory
t = 0:0.01:pi/2;
xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,...
    3-0.5*sin(t),2+0.5*cos(t),2-0.5*sin(t)];
yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
    2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];
% total landmarks numbers
num = size(landmarks,2);

% initial pose
x0 = 1;
y0 = 1;
theta0 = 0;

%% Using integrated kinetmatics to plot the trajectory and landmarks
figure
trail_axes = gca();

% plot the ground truth
plot(xTrace,yTrace,'g-','Parent',trail_axes); grid on; grid minor
hold on
title("Integrated robot position")
xlim(trail_axes,[0,5]);
ylim(trail_axes,[0,5]);
axis(trail_axes,'manual');

state = [x0;y0;theta0];
pbar = zeros(3,num);
    
% start odometry
for i = 1:size(myDataset,2)
    data = myDataset(i);
    
    wheel = data.wheelVelocity;
    velocity = data.velocity;
    wl = wheel(1); wr = wheel(2);
    u = velocity(1); q = velocity(2);
    dt = data.time;
    y = data.landmarks;
    ybar = y(1:3,:);

    state(1:3,i+1) = integrate_kinematics(state(1:3,i),dt,u,q);
    xNew = state(1,i+1); yNew = state(2,i+1); thetaNew = state(3,i+1);
    
    P = [cos(thetaNew) -sin(thetaNew) xNew;
         sin(thetaNew) cos(thetaNew) yNew;
         0 0 1];
     
    phat = P * ybar;
    pbar(:,y(4,:)) = phat;
     
    % plot the trail
    plot(state(1,:),state(2,:),'b-','Parent',trail_axes); drawnow;
    % plot the phat
    for j = 1:size(y,2)
        group = y(4,j);
        scatter(phat(1,j),phat(2,j),6,'MarkerFaceColor',cmap(group),...
            'MarkerEdgeColor','none'); hold on;
    end
    
end
    rmse = RMSE(pbar,landmarks); disp("RMSE(integration)="+num2str(rmse));
    armse = ARMSE(pbar,landmarks); disp("ARMSE(integration)="+num2str(armse));
    
%% visual odometry

% plot the ground truth
figure
trail_axes = gca();
plot(xTrace,yTrace,'g-','Parent',trail_axes); grid on; grid minor
hold on
title("Visual odometry")
xlim(trail_axes,[0,5]);
ylim(trail_axes,[0,5]);
axis(trail_axes,'manual');

observe_num = zeros(1,num);

% initial Phat
Phat = [cos(theta0) -sin(theta0) x0;
        sin(theta0) cos(theta0) y0;
        0 0 1];
% initialise for estimated p
pbar = zeros(3,num);
trail = [];

% initial gain
gain_k0 = 0.3;
gain_ki = [];
ci = [];

for i = 1:size(myDataset,2)
    data = myDataset(i);
    
    wheel = data.wheelVelocity;
    velocity = data.velocity;
    wl = wheel(1); wr = wheel(2);
    u = velocity(1); q = velocity(2);
    dt = data.time;
    y = data.landmarks; % measured landmarks with ids
    ybar = y(1:3,:);
    
    if ~isempty(y)
        observe_num(1,y(4,:)) = observe_num(1,y(4,:)) + 1;
    end
    
    W = [0 -q u;
         q 0 0;
         0 0 0];
     
    phat = Phat * ybar; 
    pClip = pbar(:,y(4,:));
    for k = 1:size(ybar,2)
        if pClip(3,k) == 0
            pClip(:,k) = phat(:,k);
        end
    end
    
    % calculate the error
    ebar = Phat * ybar - pClip;
    
    % set gain 
    [gain_k0, gain_ki, ci] = setGain(ebar, dt, y, observe_num);
    
    [Phat,phat] = observer(Phat,pClip,ebar,ybar,dt,W,gain_k0,gain_ki,ci);
    
    % update the (little) p measured position
    pbar(:,y(4,:)) = phat;
     
    % plot the trail
    trail = [trail, Phat(1:2,3)];
    plot(trail(1,:),trail(2,:),'b-','Parent',trail_axes);
    drawnow;
    % plot the phat
    for j = 1:size(y,2)
        group = y(4,j);
        scatter(phat(1,j),phat(2,j),6,'MarkerFaceColor',cmap(group),...
            'MarkerEdgeColor','none'); hold on;
    end
    
end

rmse = RMSE(pbar,landmarks); disp("RMSE(VO)="+num2str(rmse));
armse = ARMSE(pbar,landmarks); disp("ARMSE(VO)="+num2str(armse));
plot(pbar(1,:),pbar(2,:),'b*','Parent',trail_axes);
%% functions

function [Phat,phat] = observer(Phat,phat,ebar,ybar,dt,W,gain_k0,gain_ki,ci)
    Phat = Phat * expm(dt*(W-gain_k0*projector(Phat'*ebar*ci*ybar')));
    phat = phat + dt*(1-gain_k0)*ebar*gain_ki;
end

function W = projector(U)
    W = zeros(3);
    W(1:2,1:2) = 0.5*(U(1:2,1:2)-U(1:2,1:2)');
    W(1:2,3) = U(1:2,3); 
end
