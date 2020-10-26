function [u,q,void] = line_control(img, speed)
    %detected if is end of the line
    void = 0;
    
    img = rgb2gray(img);
    img = ~imbinarize(img, 0.2);
    img = img(end-59:end-30, :);
    % find the line in the image
    [rows, cols] = find(img);
    
    if isempty(rows) || isempty(cols)
        void = 1;
        u = NaN;
        q = NaN;
    end
    
    line_centre = (mean(cols)-400/2)/200;
    
    u = 0.1*speed*(1-line_centre^2);
    q = -3*line_centre;

end