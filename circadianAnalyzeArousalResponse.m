function [arousal,singlePlots]=circadianAnalyzeArousalResponse(speed,motorON,motorOFF,tElapsed)

% Inputs: speed vector (frames x flies), arrays of indices corresponding to
% times when the motor turned on and off
%
% Outputs: baseline activity for each fly, activity decay constant for each fly

meanSpd=nanmean(speed,1);                       % avg speed

%% Stimulus triggered averaging for each fly

w=3;       % width of the averaging window in min
w=w*60;     % convert min to sec

% Get parameters for the width of each window in indices
tON=tElapsed(motorON(1));
[v,iON(1)]=min(abs(tElapsed-(tON-w)));
[v,iON(2)]=min(abs(tElapsed-tON));
tOFF=tElapsed(motorOFF(1));
[v,iOFF(1)]=min(abs(tElapsed-tOFF));
[v,iOFF(2)]=min(abs(tElapsed-(tOFF+w)));

w=round(mean([diff(iON) diff(iOFF)]));  % Convert to indices
traces=NaN(w*2+2,size(motorON,2),size(speed,2));

for i=1:size(motorON,2)
traces(:,i,:)=speed([motorON(i)-w:motorON(i) motorOFF(i):motorOFF(i)+w],:);
end

% Plot of the population stimulus triggered average
figure();
popMeanPlot=smooth(mean(mean(traces,2),3),100,'lowess');
plot(popMeanPlot,'r','LineWidth',2);
hold on
plot([w w],[0 max(popMeanPlot)+1],'k','LineWidth',2);
hold off
p=findobj(gca, 'Color', 'r');
uistack(p, 'top');

% Plot of the average for each fly
singlePlots=NaN(size(traces,1),size(traces,3));
for i=1:size(singlePlots,2)
    singlePlots(:,i)=smooth(mean(traces(:,1,i),2),200,'lowess');
end

%% Display plots
%{
for h=1:4
figure();
    for i=h*24-23:h*24
      subplot(6,4,mod(i-1,24)+1);
      hold on
      plot(singlePlots(:,i),'r','Linewidth',2);
      plot([w w],[0 max(singlePlots(:,i))+1],'k','LineWidth',2);
      axis([0 size(singlePlots(:,i),1) 0 max(singlePlots(:,i))+1]);
      hold off
    end
end
%}

%% Calculate before and after activity ratio

stimAvg=mean(traces,2);
before=squeeze(mean(stimAvg(1:w,1,:)));
after=squeeze(mean(stimAvg(size(stimAvg,1)-w:end,1,:)));
arousal=after./(before+after);
