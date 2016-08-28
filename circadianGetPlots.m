function [dt,tElapsed,plotData,EvenHrIndices,timeLabels,lightON,lightOFF,motorON,motorOFF]=circadianGetPlots(motorID,speed,tStart,tON,tOFF,interval,stepSize)

%% Define parameters and variables for analysis

motorData=dlmread(motorID);

% Interval specifies the length of the sliding window in minutes and must
% be converted to milliseconds
interval=interval*60;
numFlies=size(motorData,2)-3;
tElapsed=cumsum(motorData(:,2));
stepSize=stepSize*60;
transitions=[0 diff(motorData(:,3))'];
motorON=find(transitions==1);
motorOFF=find(transitions==-1)-1;

%% Define indices for subsampling the data at the interval and step size specified

% First point should be from first index to t = interval
% firstIndex=sum(tElapsed<interval);
samplingTimes=0:stepSize:tElapsed(end);
numSteps=length(samplingTimes);
samplingIndex=NaN(size(samplingTimes));

for i=1:numSteps
   d=abs(tElapsed-samplingTimes(i));
   [v,j]=min(d);
   samplingIndex(i)=j;
   clearvars d v j
end

%% Find the nearest indices to every day and even hour transition

% Find time to day transition
tShift=tStart(4)*60*60+tStart(5)*60+tStart(6);
offsetTime=double(samplingTimes)+tShift;
daySec=24*60*60;
DayIndices=find(diff(mod(offsetTime,daySec))<-daySec+10000);

% Find even hour transitions
twoHrSec=2*60*60;
EvenHrIndices=find(diff(mod(offsetTime,twoHrSec))<-twoHrSec+1000);

% Generate strings for plot labels
timeLabels=cell(length(EvenHrIndices),1);
DayIndices=[DayIndices max(EvenHrIndices)+1];
ct=1;
for i=1:length(DayIndices)
    tmpHrInd=EvenHrIndices(EvenHrIndices<=DayIndices(i));
    EvenHrIndices(1:length(tmpHrInd))=[];
    Hrs=flip(24:-2:24-2*(length(tmpHrInd)-1));
        if i==length(DayIndices)
        Hrs=2:2:2*(length(tmpHrInd));
        end
    for j=1:length(Hrs)
    timeLabels(ct)={[num2str(Hrs(j)) ':00']};
    ct=ct+1;
    end
end
 EvenHrIndices=find(diff(mod(offsetTime,twoHrSec))<-twoHrSec+1000);
 
%% Find indices corresponding to lights on and lights off times

% Determine if experiment started in the light or in the dark

if tStart(4)>=tON(1) && tStart(4)<=tOFF(1)
    if tStart(4)<tOFF(1)
        lightStat=1;
    elseif tStart(4)==tOFF(1)
        if tStart(5)<tOFF(2)
            lightStat=1;
        else
            lightStat=0;
        end 
    end
else
    lightStat=0;
end

% Find the indices corresponding to changes
timeON2OFF=(tOFF(1)-tON(1))*60*60 + (tOFF(2)-tON(2))*60;
timeOFF2ON=24*60*60-timeON2OFF;
lightOFF=[];
lightON=[];

if lightStat
    t=tOFF(1)*60*60+tOFF(2)*60;
    [v,iOFF]=min(abs(offsetTime-((t))));
    lightON=1;
    lightOFF=iOFF;
    while t < offsetTime(end)
        t=t+timeOFF2ON;
        if t <= offsetTime(end)
            [v,iON]=min(abs(offsetTime-((t))));
            lightON=[lightON iON];
        end
        t=t+timeON2OFF;
        if t <= offsetTime(end)
            [v,iOFF]=min(abs(offsetTime-((t))));
            lightOFF=[lightOFF iOFF];
        end
    end
    if length(lightON)>length(lightOFF)
        lightOFF=[lightOFF length(offsetTime)];
    end
else
    t=tON(1)*60*60+tON(2)*60;         % Calculate first lights ON time
    
    % If the exp started in the evening, shift time by one day
    if offsetTime(1) > 12*60*60
        t=t+24*60*60;
    end
    
    [v,iON]=min(abs(offsetTime-((t))));
    lightON=[iON];
    lightOFF=1;
    while t < offsetTime(end)
        t=t+timeON2OFF;
        if t <= offsetTime(end)
            [v,iOFF]=min(abs(offsetTime-((t))));
            lightOFF=[lightOFF iOFF];
        end
        t=t+timeOFF2ON;
        if t <= offsetTime(end)
            [v,iON]=min(abs(offsetTime-((t))));
            lightON=[lightON iON];
        end
    end
    if length(lightON)<length(lightOFF)
        lightON=[lightON length(offsetTime)];
    end
end
    
%% Convert light ON/OFF times into vertices for light region shading in plots

if lightON(1) > lightOFF(1)
    lightOFF(1)=[];
    if isempty(lightOFF)
        lightOFF=length(offsetTime);
    elseif length(lightOFF)~=length(lightON)    
        lightOFF(end)=length(offsetTime);
    end
end


%% Slide window over speed data
interval_step_num=ceil(interval/stepSize);
plotData=zeros(length(samplingIndex)-interval_step_num,numFlies);
n=length(samplingIndex)-interval_step_num;
f=round(n/10);
for i=1:n
    tic
    plotData(i,:)=nanmean(speed(samplingIndex(i):samplingIndex(i+interval_step_num)-1,:));
    if mod(i,f)==0
    disp([num2str(((n-i)*toc)) ' estimated seconds remaining']);
    end
end

%%
% Find indices from sampling data closest to motor transitions
mi1=NaN(size(motorON));
mi2=NaN(size(motorOFF));

for i=1:length(motorON);
    d=abs(samplingIndex-motorON(i));
    [v,j]=min(d);
    mi1(i)=j;
    d=abs(samplingIndex-motorOFF(i));
    [v,j]=min(d);
    mi2(i)=j;
end   


%% Plot population mean over time overlayed with motorpulse times and light dark transitions   

plotData(1,:)=[];
meanPlot=nanmean(plotData,2);
w=ceil(size(meanPlot,1)/100)
smoothedPlot=smooth(meanPlot',w,'lowess');

% Plot light patches
figure();
yLim=max(meanPlot)+1;
hold on
for i=1:length(lightON)
    x=[lightON(i) lightOFF(i) lightOFF(i) lightON(i)];
    y=[0 0 yLim yLim];
    patch(x,y,'yellow','FaceAlpha',0.4);
end

plot(smoothedPlot,'r','LineWidth',2);
set(gca,'Xtick',EvenHrIndices,'XtickLabel',timeLabels)
for i=1:length(mi2);
    plot([mi1(i) mi1(i)],[0 yLim],'Color',[0 1 1],'Linewidth',2);
end

activityTrace = findobj(gca, 'Color', 'r');
uistack(activityTrace, 'top')
axis([0 size(meanPlot,1) 0 yLim])
