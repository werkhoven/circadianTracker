function out=avgAngle_legacy(dataCluster,roi)

% out=avgAngle(dataCluster,roi)

numFlies=length(dataCluster);
speedLowerBound=1.4;
speedUpperBound=24.5;

for i=1:numFlies
    fly=dataCluster(i);
    angle=fly.theta(fly.speed>speedLowerBound & fly.r<roi(i) & fly.speed<speedUpperBound)...
        -fly.direction(fly.speed>speedLowerBound & fly.r<roi(i) & fly.speed<speedUpperBound);
    
    %% Calculate a mu score for each fly

    % Shift all angles to be between 0 and 2pi
    for j=1:length(angle)
        if angle(j)<0
            angle(j)=angle(j)+(2*pi);
        end
    end
    
    % Bin data into angle histograms, later used to calculate occupancy
    bins=0:2*pi/26:2*pi;
    ha=histc(angle,bins);
    ha=ha/length(angle);
    ha(end)=[];
    out(i).angleavg=ha;
    bins(end)=[];
    theta_d_weight=-sin(bins+mean(diff(bins))/2)';
    out(i).mu=sum(ha.*theta_d_weight);
end

