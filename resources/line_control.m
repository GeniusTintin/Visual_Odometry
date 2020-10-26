function [u, q] = line_control(img, speed_factor)

gray_img = rgb2gray(img);
bin_img = ~imbinarize(gray_img, 0.2);

[r, c] = find(bin_img(150:180,:) == 1);
centre_of_mass = [mean(c), mean(r)];
centre_of_mass = (centre_of_mass - [200, 100]) ./ [200, 100];

% Find the centre of the line to follow
line_centre = centre_of_mass(1); % replace with correct value

% Use the line centre to compute a velocity command
q = -speed_factor * 0.8*line_centre;
% Drive forward as soon as the dot is roughly in view
u = speed_factor * 0.1;

end

