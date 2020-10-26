close all
addpath("../simulator");
% set landmarks on the scene
landmarks = 2*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)]+[2.5;2.5];
landmarks = [landmarks, 1.5*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)]+[2.5;2.5]];
pb = piBotSim('floor_circle.jpg', landmarks);
% measure the landmark only in the green rigion.  

pb.place([2.5;1.5],0);
% robot pose (homogeneous pose matrix)
Phat = [1 0 2.5;
        0 1 1.5;
        0 0 1];

dt = 0.1;

figure;
trail_axes = gca();
trail = [];
gain_k = 0.02;

while true
    img = pb.getCamera();
    [u,q] = line_control(img, 5.0);
    [wl, wr] = inverse_kinematics(u, q);
    pb.setVelocity(wl, wr);
    
    % integrate the kinematics using the matrix exponential
    % Integrate \dot{P} = P W;
    W = [0 -q u;
         q 0 0;
         0 0 0];
    Phat = Phat * expm(dt * W);
    
    % Compute innovation
    % first take a measurement
    [lms, ids] = pb.measureLandmarks();
    ybar = [lms; ones(1, numel(ids))]; % numel retuen the number of elements
    ebar = Phat * ybar;
    Pbar = [landmarks(:, ids); ones(1,numel(ids))]; % the measured pose
    Delta = gain_k * projector((ebar-Pbar)*ebar');
    
    % Integrate \dot{P} = -Delta P
    Phat = expm(-dt * Delta) * Phat ; 
    
    % save the intergated position to the trail
    trail = [trail, Phat(1:2,3)];
    plot(trail(1,:), trail(2,:),'b-','Parent', trail_axes);
    % limit the scale
    xlim(trail_axes, [0,5]);
    ylim(trail_axes, [0,5]);
    axis(trail_axes, 'equal');
end

function W = projector(U)
    %design of a projector matrix
    W = zeros(3);
    W(1:2,1:2) = 0.5*(U(1:2,1:2)-U(1:2,1:2)');
    W(1:2,3) = U(1:2,3);
end
