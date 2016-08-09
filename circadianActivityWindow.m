function circData=circadianActivityWindow(interval,stepSize)

%% Get paths to data files
[fName,fDir,fFilter] = uigetfile('*.txt;*','Open data file',...
    'C:\Users\debivort\Documents\MATLAB\Decathlon Raw Data','Multiselect','on');
tic
motorData=dlmread(strcat(fDir,fName));
toc
% Interval specifies the length of the sliding window in minutes and must
% be converted to milliseconds
interval=interval*60;
numFlies=size(motorData,2)-3;
tElapsed=cumsum(motorData(:,2));
stepSize=stepSize*60;
speed=motorData(:,4:size(motorData,2));
transitions=[0 diff(motorData(:,3))'];
motorON=find(transitions==1);
motorOFF=find(transitions==-1)-1;

% First point should be from first index to t = interval
%firstIndex=sum(tElapsed<interval);
samplingTimes=0:stepSize:tElapsed(end);
numSteps=length(samplingTimes);
samplingIndex=NaN(size(samplingTimes));

for i=1:numSteps
   d=abs(tElapsed-samplingTimes(i));
   [v,j]=min(d);
   samplingIndex(i)=j;
end

% Slide window over speed data
interval_step_num=round(interval/stepSize);
plotData=zeros(length(samplingIndex)-interval_step_num,numFlies);
n=length(samplingIndex)-interval_step_num;
f=round(n/100);
for i=1:n
    tic
    plotData(i,:)=nanmean(speed(samplingIndex(i):samplingIndex(i+interval_step_num)-1,:));
    if mod(i,f)==0
    disp([num2str(((numSteps-i)*toc)*60) ' estimated min remaining']);
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


%% Plot population mean over time and plot
meanPlot=nanmean(plotData,2);
meanPlot(1)=[];

plot(meanPlot,'r');
hold on
for i=1:length(mi2);
    plot([mi1(i) mi1(i)],[0 max(meanPlot)+1],'Color',[0 1 1],'Linewidth',2);
end

activityTrace = findobj(gca, 'Color', 'r');
uistack(activityTrace, 'top')

%%
plotData(1,:)=[];

%{
for h=1:4
hold on
figure();
    for i=h*24-23:h*24
      subplot(6,4,mod(i-1,24)+1);
      hold on
      plot(smooth(plotData(:,i),ceil(length(plotData)/50)),'r');
        for j=1:length(mi2);
            hold on
            plot([mi1(j) mi1(j)],[0 2],'Color',[0 0 1],'Linewidth',1);
        end
      axis([0 size(plotData,1) 0 2]);
    end
end
%}
circData.tElapsed=tElapsed;
circData.activeFlies=nanmean(speed)>0.01;
circData.numActive=sum(circData.activeFlies);
circData.speed=speed;
circData.motorON=motorON;
circData.motorOFF=motorOFF;







