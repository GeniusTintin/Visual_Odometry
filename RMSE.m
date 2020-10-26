function rmse = RMSE(phat,landmarks)
% ------inputs------
% phat: currently observed landmarks positions
% landmarks: selected ground truth data corresponds to the observed landmarks
    n = size(phat,2);
    p = phat(1:2,:);
    
%   rmse = 0;
%     for i = 1:n
%         rmse = rmse + (norm(p(:,i)-landmarks(:,i)))^2;
%     end
%     rmse = sqrt(rmse/n);
    rmse = sqrt(sum((p-landmarks).^2,'all')/n);
end
