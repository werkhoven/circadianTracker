function out=circAngle(cData,widths)

% out=avgAngle(dataCluster,roi)

numFlies=length(cData);

bins=0:2*pi/25:2*pi;

haall=zeros(numFlies,length(bins)-1);

for i=1:numFlies
    fly=cData(i);
    tmp_valid=fly.speed>0.8 & fly.speed<0.25*widths(i); %& fly.r<0.5*widths(i);
    s=fly.speed;
    r=fly.r;
    a=fly.theta;
    da=[0;diff(a)];
    da=da(tmp_valid);
    s=s(tmp_valid);
    r=r(tmp_valid);
    da(da>5)=da(da>5)-2*pi;
    da(da<-5)=da(da<-5)+2*pi;
    mu=(da.*r./s);
    mu(isinf(mu))=NaN;
    mu=nanmean(mu);
    angle=NaN(size(fly.speed));
    angle(tmp_valid)=fly.theta(tmp_valid)-fly.direction(tmp_valid);
    angle(angle<0)=angle(angle<0)+(2*pi);
    ha=histc(angle,bins);
    ha=ha/sum(ha);
    ha(1)=ha(1)+ha(end);
    ha(end)=[];
    flyCircles(i).angleavg=single(ha/sum(ha));
    flyCircles(i).valid_trials=logical(tmp_valid);
    flyCircles(i).circum_vel=single(angle);
    flyCircles(i).mu=mu;
    flyCircles(i).circum_vel=single(fly.theta-fly.direction);
end

for i = 1:numFlies
    
    flyCircles(i).angleSD = std(flyCircles(i).angleavg);
    flyCircles(i).angleSEM = std(flyCircles(i).angleavg)/sqrt(sum(flyCircles(i).valid_trials));
    
end

out = flyCircles

