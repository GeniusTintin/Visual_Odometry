%% data_collection
close all
clear all
clc
addpath("../simulator/");
% landmarks alone the course
% landmarks1 = [(1:0.5:3.5);ones(1,6)];
% landmarks2 = [[3.5;1.5],[4;2],[4.5;2.5]];
% landmarks3 = [(4.5:-0.5:2.5);3*ones(1,5)];
% landmarks4 = [(1.5:0.5:2.5);(1.5:0.5:2.5)];
% landmarks5 = [4.5;2];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks alone the course less
% landmarks1 = [2 3;ones(1,2)];
% landmarks2 = [[3.5;1.5],[4;2],[4.5;2.5]];
% landmarks3 = [(4.1:-1:3.1);3*ones(1,2)];
% landmarks4 = [(2.5:-0.5:1.5);(2.5:-0.5:1.5)];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4];

% landmarks alone the curve
% landmarks1 = [2;1];
% landmarks2 = 0.5*[cos(-pi/3:pi/6:0);sin(-pi/3:pi/6:0)] + [3;1.5];
% landmarks3 = 0.5*[cos(-pi/2:pi/6:pi/2);sin(-pi/2:pi/6:pi/2)] + [4;2.5];
% landmarks4 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [3;2.5];
% landmarks5 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [2;1.5];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks along the line
% landmarks1 = [1:0.5:3;ones(1,5)];
% landmarks2 = 0.5*[cos(3*pi/4);sin(3*pi/4)] + [4;1.5];
% landmarks3 = [4.5;2.5];
% landmarks4 = [2.5:0.5:4;3*ones(1,4)];
% landmarks5 = [2;2];
% landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks alone the course (more)
landmarks1 = [1.5:1:2.5;ones(1,2)];
landmarks2 = 0.5*[cos(-pi/3:pi/6:0);sin(-pi/3:pi/6:0)] + [3;1.5];
landmarks3 = 0.5*[cos(-pi/2:pi/6:pi/2);sin(-pi/2:pi/6:pi/2)] + [4;2.5];
landmarks4 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [3;2.5];
landmarks5 = 0.5*[cos(pi/2:pi/6:pi);sin(pi/2:pi/6:pi)] + [2;1.5];
landmarks = [landmarks1,landmarks2,landmarks3,landmarks4,landmarks5];

% landmarks grid
% [lmx,lmy] = meshgrid(0.5:(4/3):4.5);
% landmarks = [lmx(:)'; lmy(:)'];

save('./Dataset/landmarks_more.mat','landmarks');

pb = piBotSim("floor_course.jpg",landmarks);

x0 = 1;
y0 = 1;
theta0 = 0;

pb.place([x0;y0],theta0);

speed = 2;

myDataset = [];
start_time = tic;
while true
    img = pb.getCamera();
    gray = rgb2gray(img);
    bin_img = ~imbinarize(gray, 0.2);
    
    imgB = bin_img(end-59:end-30,:);
    [rows, cols] = find(imgB);
    
    if isempty(rows)||isempty(cols)
       u = 0.1;
       [wl, wr] = inverse_kinematics(u, 0);
       pb.setVelocity([wl, wr], 0.3);
       break;
    end
    
    line_centre = (mean(cols)-200)/200;
    u = 0.1*speed*(1-line_centre^2);
    q = -pi*line_centre;
    [wl,wr] = inverse_kinematics(u, q);   
    [lms, ids] = pb.measureLandmarks();
   
    clc
    % evaluate each time step
    elapsed_time = toc(start_time)
    pb.setVelocity(wl, wr);
    start_time = tic;
    
    data.velocity = [u;q];
    data.wheelVelocity = [wl;wr];
    data.landmarks = [lms;ones(1,numel(ids));ids];
    data.time = elapsed_time;
    myDataset = [myDataset,data];
end

% the timestep is one step behind 
size = size(myDataset,2);
for i=1:size-1
    myDataset(i).time = myDataset(i+1).time;
end
myDataset = myDataset(1:end-1);

save('./Dataset/myDataset_curve.mat', 'myDataset');
save('./Dataset/landmarks_curve.mat','landmarks');
