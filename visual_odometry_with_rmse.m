% Always begin by using addpath
addpath("../simulator")

% For testing, we can tell the simulator where we want to place our
% landmarks. Let's try a grid formation
[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];
load("./Dataset/landmarks_square.mat")
% Now, we can start the simulator with these landmarks.
% You can also try set your own landmarks, or, if you leave it blank, the
% simulator will generate landmarks randomly.
pb = piBotSim("floor_spiral.jpg",landmarks);
pb.showLandmarks(true); % Use this to turn on/off showing the landmarks.
pb.showViewRange(true); % Use this to turn on/off showing the pibot view.

% Place your robot at the centre of the room
% initial pose
x0 = 2.5;
y0 = 4.5;
theta0 = 180;
pb.place([x0;y0],theta0);

% Which landmarks can we measure? Try the measurement function
[lms, ids] = pb.measureLandmarks();
% How accurate are these measurements? While testing, you can know where 
% the landmarks are, so you can check what kind of error the measurements
% have. How is the error distributed?


% Now you can use these measurements in your odometry observer!
% You will also need to use your known input velocity to the robot.
% I strongly suggest you try some simple examples before you try to follow
% a line, e.g. drive in a straight line for a few seconds.
% This will also help to evaluate your solution!

%% ground truth trajectory
t = 0:0.01:pi/2;
xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,...
    3-0.5*sin(t),2+0.5*cos(t),2-0.5*sin(t)];
yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
    2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];

%% total landmarks numbers
num = size(landmarks,2);

%% visual odometry

% plot the ground truth
figure
trail_axes = gca();
%plot(xTrace,yTrace,'g-','Parent',trail_axes); grid on; grid minor
hold on
title("Visual odometry")
xlim(trail_axes,[0,5]);
ylim(trail_axes,[0,5]);
axis(trail_axes,'manual');

observe_num = zeros(1,num);
dt = 0.1;

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

t = 0;
while true
    img = pb.getCamera();
    [u,q,void] = line_control(img, 2.0);
    if void
        break;
    end
    
    [wl,wr] = inverse_kinematics(u,q);
    
    pb.setVelocity(wl,wr);
    t = t + dt;
    
    W = [0 -q u;
         q 0 0;
         0 0 0];
    
    
    [lms, ids] = pb.measureLandmarks();

    y = [lms;ones(1,numel(ids));ids];

    ybar = y(1:3,:);
    
    if ~isempty(y)
        observe_num(1,y(4,:)) = observe_num(1,y(4,:)) + 1;
    end
    
    
     
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
    if t>300
        break;
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