function out=avgAngle_legacy(dataCluster,roi,area)

% out=avgAngle(dataCluster,roi)

numFlies=length(dataCluster);
speedLowerBound=1.1;
speedUpperBound=24.5;
avg_area=nanmean(area);

for i=1:numFlies
    fly=dataCluster(i);
    
    spd_mask=fly.speed>speedLowerBound & fly.speed<speedUpperBound;
    wdth_mask=fly.r<roi(i);
    ceil_mask=area(:,i)>avg_area(i);
    floor_mask=area(:,i)<=avg_area(i);
    
    % Partition data into floor and ceiling bouts
    angle_top=fly.theta(spd_mask & wdth_mask & ceil_mask)-fly.direction(spd_mask & wdth_mask & ceil_mask);
    angle_bottom=fly.theta(spd_mask & wdth_mask & floor_mask)-fly.direction(spd_mask & wdth_mask & floor_mask);
    
    % Flip angle of ceiling walking where the fly is upside down
    angle_top=-angle_top;
    
    % Recombine into single angle vector
    angle=NaN(size(spd_mask));
    angle(spd_mask & wdth_mask & ceil_mask)=angle_top;
    angle(spd_mask & wdth_mask & floor_mask)=angle_bottom;
    angle(isnan(angle))=[];
    
    %% Calculate a mu score for each fly

    % Shift all angles to be between 0 and 2pi
    angle(angle<0)=angle(angle<0)+(2*pi);
    
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

