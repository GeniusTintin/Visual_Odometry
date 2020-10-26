function [k0,ki,ci] = setGain(ebar,dt,y,observe_num)
    k0 = 0.15;
    ki = zeros(size(y,2)); %number of detected landmarks
    ci = zeros(size(y,2)); 
    kt = 1/dt; %get rid of the time effect
     
    numClip = observe_num(1,y(4,:));
    
    distance = zeros(1,size(y,2));
    
    % measure the distance of each observed point
    for i = 1:size(ebar,2)
        distance(i) = norm((ebar(1:2,i)-[0;0]));
    end
    
    if ~isempty(y)
       for i = 1:size(y,2)
          distance_gain = (1-(distance(i))/max(distance,[],2));
          if max(distance,[],2)==0
              distance_gain = 1;
          end
          ki(i,i) = kt * (1.5/(numClip(i))^(1/2) *1);    % ki gains for phat observer
          %ki(i,i) = 0;
          if norm(ebar(1:2,i))==0
              ci(i,i) = 1;
          else
              %ci(i,i) = 0.08 * norm(ebar(1:2,i));
              ci(i,i) = 0.8;
              %ci(i,i) = 1/size(y,2);
          end
       end
    end
    
end


%% tested gains
%{
    ki(i,i) = -(1/(1-k0)) * kt*(1/log10(9.9+0.1*numClip(i)));
    ki(i,i) = -(1/(1-k0)) * kt*1/numClip(i); 

    ci(i,i) = 0.05*norm(y(1:2,i));

    best:
    k0 = 0.15
    ki(i,i) = kt * 1/sqrt(numClip(i));
    ci(i,i) = 0.8;

    curve:
    ci(i,i) = 0.02 * norm(ebar(1:2,i)) * sqrt(numClip(i));

    line trajectory regression
    ki(i,i) = kt * 1/numClip(i); 
    ci(i,i) = 0.02 * norm(ebar(1:2,i));
%}