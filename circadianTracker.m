
%NB: needs the function find96WellPlate() in the same folder to work
%as well as image toolbix
%and symbolic math toolbox

clearvars -except handles

%% Execute this section to initialize workspace with UI variables

%{
handles.fpath=...
'C:\Users\debivort\Documents\MATLAB\Decathlon Raw Data\Testing';
handles.exp_duration=0.01;
handles.pulse_interval=1;
handles.pulse_number=5;
handles.pulse_frequency=1;
handles.labels=cell(1,8);
handles.labels(:)={''};
handles.labels(1,1)={'testStrain'};
handles.ref_freq=120;
handles.ref_stack_size=3;
handles.tracking_thresh=22.3;

%Close and delete any open serial objects
if ~isempty(instrfindall)
fclose(instrfindall);           % Make sure that the COM port is closed
delete(instrfindall);           % Delete any serial objects in memory
end

% Attempt handshake with light panel teensy
[handles.teensy_port,ports]=identifyMicrocontrollers;

camInfo=imaqhwinfo('pointgrey');
handles.camInfo=camInfo;
handles.Cam_mode=handles.camInfo.DeviceInfo.SupportedFormats(3);

imaqreset;
pause(0.2);
handles.vid=initializeCamera(handles.camInfo.AdaptorName,handles.camInfo.DeviceIDs{1},handles.Cam_mode{:});
%}

%% INPUT VARIABLES

%refStackUpdateTiming=1; % How often to update a ref image, so all ref images updated 7*5 = 35sec
imagingLayer=2; %Channel of the RGB camera to be extracted to follow the movements (green channel, i.e. imagingLayer=1, works the best)
refStackUpdateTime=1/handles.ref_freq; %in s

%Duration of experiment, vibration pattern
handles.exp_duration=handles.exp_duration*60*60; % Convert length of the experiment from hours to seconds
handles.pulse_interval=handles.pulse_interval*60; % Convert min to sec

%Fly position extraction
ROIScale=0.9/2;
speedThresh=55;

%% Save labels and create placeholder files for data

tStart = datestr(clock,'mm-dd-yyyy-HH-MM-SS_');
labels = cell2table(labelMaker(handles.labels),'VariableNames',{'Strain' 'Sex' 'Treatment' 'ID' 'Day'});
strain=labels{1,1}{:};
treatment=labels{1,3}{:};
labelID = [handles.fpath '\' tStart strain '_' treatment '_labels.dat'];         % File ID for label data
writetable(labels, labelID);                                                % Save label table

% Create placeholder files
cenID = [handles.fpath '\' tStart strain '_' treatment '_Centroid.dat'];         % File ID for centroid data
motorID = [handles.fpath '\' tStart strain '_' treatment '_Motor.dat'];          % File ID for motor data
 
dlmwrite(cenID, []);                                                        % create placeholder ASCII file
dlmwrite(motorID, []);                                                        % create placeholder ASCII file

%% Initialize dialogue with Teensy

% Close and delete any open serial objects
if ~isempty(instrfindall)
fclose(instrfindall);           % Make sure that the COM port is closed
delete(instrfindall);           % Delete any serial objects in memory
end

%Initialize vibration parameters
isVibrationOn=0;                                    % Trackings whether current iteration is during a bout of stimulation
isPulseOn=0;                                        % Pulse state from current iteration
wasVibrationOn=0;                                   % tracks whether previous iteration occured during bout of stimulation
wasPulseOn=0;                                       % Pulse state from previous iteration
howManyPulses=0;                                    % Current num. pulses that have occured during a bout of stimulation
howManyVibrations=0;                                % Current num. pulses that have occured during the entire experiment
pulseTime=(1/handles.pulse_frequency)/2;            % Length of time between successive pulses during same bout
pulseDur=(1/handles.pulse_frequency)/2;             % Length of time any given pulse is on
vibrationTime=handles.pulse_interval;
vibrationDur=handles.pulse_number*(1/handles.pulse_frequency);

%Initialize output matrix
out=[];
% 'out' is a 2D matrix.
% Number of lines: one per time point. There are as many lines as the
% variable 'counter' at the end of the experiment.
% Number of columns: 196.
%
% First column: Time elapsed since the beginning of
% the experiment (variable 'tElapsed'). Second column: Cumulated instant
% speed for all the wells. Third column: Standard deviation of the instant
% speed calculated through all the wells. Fourth column: 1 if vibration is
% on, 0 if not. From columns 5 to 192: coordinates of the centroid detected
% in each well, [X(well 1) Y(well 1) X(well 2)...].
% 
% For second and third columns: If no centroid is detected, then it takes
% the value NaN. For the columns 5 to 192: If there is no movement detected
% in the well i, then X(well i)=NaN and Y(well i)=NaN

%% Determine position in light/dark cycle and initialize white light

t=clock;            % grab current time
t=t(4:5);           % grab hrs and min only

if handles.lights_ON(1)<=t(1) && handles.lights_OFF(1)>=t(1)
    if handles.lights_ON(2)<=t(2)
        writeInfraredWhitePanel(handles.teensy_port,0,handles.White_intensity);
        lightStatus=1;
    else
        writeInfraredWhitePanel(handles.teensy_port,0,0);
        lightStatus=0;
    end
else
        writeInfraredWhitePanel(handles.teensy_port,0,0);
        lightStatus=0;
end

t
handles.lights_ON
handles.lights_OFF
lightStatus

%% GET PLATE COORDINATES

im=peekdata(handles.vid,1); % acquire image from camera
im=squeeze(im(:,:,imagingLayer)); % Extracting the green channel (works the best)
refImage=im;
refStack=repmat(refImage,1,1,handles.ref_stack_size);
fig=imshow(refImage);


%% Select tracking threshold

stop=boolean(0);
tic
prev_tStamp=0;

while ~stop
    
    tElapsed=toc;
    im=peekdata(handles.vid,1); % acquire image from camera
    im=squeeze(im(:,:,imagingLayer)); % Extracting the green channel (works the best)
    diffIm=refImage-im;
    pause(0.00001);
    
    % Extract centroids
    
    handles.tracking_thresh=get(handles.threshold_slider,'value');
    props=regionprops((diffIm>handles.tracking_thresh),'Centroid','Area');
    validCentroids=([props.Area]>6&[props.Area]<120);
    cenDat=reshape([props(validCentroids).Centroid],2,length([props(validCentroids).Centroid])/2)';
    
    % Display thresholded image and plot centroids
    %imshow(diffIm>handles.tracking_thresh);
    cla reset
    imagesc(diffIm>handles.tracking_thresh, 'Parent', handles.axes1);
    set(handles.edit_frame_rate,'String',num2str(1/(tElapsed-prev_tStamp)));
    
    hold on
    plot(cenDat(:,1),cenDat(:,2),'o');
    hold off
    stop=boolean(get(handles.accept_thresh_pushbutton,'value'));
    
    prev_tStamp=tElapsed;
    
end

handles.tracking_thresh
set(handles.accept_thresh_pushbutton,'value',0);

%% Detect well positions and permute well numbers to match 96 well plate labels

colormap('gray');
imagesc(im)
[xPlate,yPlate]=getline();

isPlateGridPlotted=0;

hold on
foundWells=find96WellPlate(im,0,xPlate,yPlate);

% Permute well number to match 96 well plate
permute=fliplr(reshape(96:-1:1,8,12)');
permute=permute(:);
foundWells.coords=foundWells.coords(permute,:);
colScale=foundWells.colScale;
ROISize=round((colScale/7)*ROIScale);
ROI_centers=round(foundWells.coords);
lastCentroid=ROI_centers;

centStamp=zeros(size(ROI_centers,1),1);
prevCentroids=lastCentroid;


%% Collect noise statistics and display sample tracking before initiating experiment

ct=1;                               % Frame counter
pixDistSize=100;                    % Num values to record in pixDist
pixelDist=NaN(pixDistSize,1);       % Distribution of total number of pixels above image threshold
tic
tElapsed=toc;

while ct<pixDistSize;
                
               tElapsed=toc;
               set(handles.edit_frame_rate, 'String', num2str(pixDistSize-ct));

               % Get centroids and sort to ROIs
               imagedata=peekdata(handles.vid,1);
               imagedata=imagedata(:,:,imagingLayer);
               diffIm=refImage-imagedata;

               % Extract regionprops and record centroid for blobs with (4 > area > 120) pixels
               props=regionprops((diffIm>handles.tracking_thresh),'Centroid','Area');
               validCentroids=([props.Area]>6&[props.Area]<120);
               
               % Keep only centroids satisfying size constraints and reshape into
               % ROInumber x 2 array
               cenDat=reshape([props(validCentroids).Centroid],2,length([props(validCentroids).Centroid])/2)';
               oriDat=zeros(size(cenDat,1),1);
               
               % Sort centroids to their respective ROIs
               [lastCentroid,centStamp]=...
                    optoMatchCentroids2Wells(cenDat,ROI_centers,speedThresh,ROISize,lastCentroid,centStamp,tElapsed);
                
               %Update display
               cla reset
               imagesc(diffIm>handles.tracking_thresh, 'Parent', handles.axes1);
               hold on
               % Mark centroids
               plot(lastCentroid(:,1),lastCentroid(:,2),'o','Color','r');
               hold off
               drawnow
               
               
           % Create distribution for num pixels above imageThresh
           % Image statistics used later during acquisition to detect noise
           pixelDist(mod(ct,pixDistSize)+1)=nansum(nansum(imagedata>handles.tracking_thresh));
           ct=ct+1;

end

% Record stdDev and mean without noise
pixStd=nanstd(pixelDist);
pixMean=nanmean(pixelDist);

%% MAIN EXPERIMENTAL LOOP

% Initialize Time Variables
tic
counter=1;
ref_counter=1;
tElapsed=toc;
prev_tStamp=toc;
timeBeginning=now;
tSinceLastRefUpdate=0;
pixDev=ones(10,1);                                   % Num Std. of aboveThresh from mean
noiseCt=1;                                           % Frame counter for noise sampling
centStamp=zeros(size(ROI_centers,1),1);

while tElapsed < handles.exp_duration

    % Update timestamp
    tElapsed=toc;
    pause(0.03);
 
        %Camera display    
        im=peekdata(handles.vid,1);         % acquire image from camera
        im=im(:,:,imagingLayer);            % extract imagingLayer channel
        diffIm=(refImage-im);               % Take difference image
        
        aboveThresh(mod(counter,10)+1)=sum(sum(diffIm>handles.tracking_thresh));
        pixDev(mod(counter,10)+1)=(nanmean(aboveThresh)-pixMean)/pixStd;
        
        % Reset references if noise threshold is exceeded
        if mean(pixDev)>8 || (wasVibrationOn-isVibrationOn==1)
           refStack=repmat(im(:,:,1),1,1,handles.ref_stack_size);
           refImage=uint8(mean(double(refStack),3));
           aboveThresh=ones(10,1)*pixMean;
           pixDev=ones(10,1);
           disp('NOISE THRESHOLD REACHED, REFERENCES RESET')
        end
        
        % Only gather centroids and record turns if noise is below
        % threshold
        if pixDev(mod(ct,10)+1)<4 && ~isVibrationOn

            %Detect every ref frame update
            if toc-tSinceLastRefUpdate>=refStackUpdateTime
                ref_counter=ref_counter+1;
                refStack(:,:,mod(ref_counter,handles.ref_stack_size)+1)=im;
                refImage=uint8(median(double(refStack),3)); % the actual ref image displayed is the median image of the refstack
                %disp(['mean refImage brightness is ' num2str(mean(mean(refImage)))]);
                %%%foundWells=find96WellPlate(im,0,xPlate,yPlate);
                wellCoordinates=round(foundWells.coords);
                colScale=foundWells.colScale;
                ROISize=round((colScale/7)*ROIScale);
                if isPlateGridPlotted==0
                    isPlateGridPlotted=1;
                    %figure(1)
                    hold on
                    %%plot([xPlate;xPlate(1)],[yPlate;yPlate(1)],'c')
                    %figure(2)
                    hold on
                end
                tSinceLastRefUpdate=toc;
            end

            %% Automatically re-detect the well positions after a motor pulse
            if wasPulseOn-isPulseOn==1
                pack
                refImage=im;
                tSinceLastRefUpdate=toc;
            end

                % Extract image properties and exclude centroids not satisfying
                % size criteria
                props=regionprops((diffIm>handles.tracking_thresh),'Centroid','Area');
                validCentroids=([props.Area]>6&[props.Area]<120);
                cenDat=reshape([props(validCentroids).Centroid],2,length([props(validCentroids).Centroid])/2)';

                % Match centroids to ROIs by finding nearest ROI center
                [lastCentroid,centStamp]=...
                    optoMatchCentroids2Wells(cenDat,ROI_centers,speedThresh,ROISize,lastCentroid,centStamp,tElapsed);
        end

        % Update the display
            
        % Write data to the hard drive
        dlmwrite(cenID, lastCentroid', '-append');

        format long
        %out(counter,:)=[mod(toc,100) NaN NaN isVibrationOn*isPulseOn reshape(centroidsTemp',1,96*2)];
        dist=sqrt((lastCentroid(:,1)-prevCentroids(:,1)).^2+(lastCentroid(:,2)-prevCentroids(:,2)).^2);
        dt=tElapsed-prev_tStamp;
        speeds=dist./dt;
        speeds=speeds*8.6/(colScale/7); %convert pixel speeds to mm/s
        dlmwrite(motorID,[counter dt isVibrationOn speeds'],'-append','delimiter','\t','precision',6);

        prevCentroids=lastCentroid;

    
    %% Vibrations - turns vibrations on and off at the correct times
    wasVibrationOn=isVibrationOn;
    if tElapsed>vibrationTime*(howManyVibrations+1) && tElapsed<vibrationTime*(howManyVibrations+1)+vibrationDur
        isVibrationOn=1;
        wasPulseOn=isPulseOn;
        if tElapsed>vibrationTime*(howManyVibrations+1)+pulseTime*(howManyPulses) && tElapsed<vibrationTime*(howManyVibrations+1)+pulseTime*(howManyPulses)+pulseDur
            isPulseOn=1;
        else
            isPulseOn=0;
            if wasPulseOn-isPulseOn==1
                howManyPulses=howManyPulses+1;
            end
        end
        
    else
        howManyPulses=0;
        isVibrationOn=0;
        if wasVibrationOn-isVibrationOn==1
            howManyVibrations=howManyVibrations+1;
        end
    end
    if wasVibrationOn-isVibrationOn==-1 && tElapsed<handles.exp_duration-0.5
        disp('VIBRATING')
        writeVibrationalMotors(handles.teensy_port,6,handles.pulse_frequency,handles.pulse_interval,...
            handles.pulse_number,handles.pulse_amplitude);
    end
    
    %% Update light/dark cycle
    
    t=clock;            % grab current time
    t=t(4:5);           % grab hrs and min only

    
    if lightStatus==1 && t(1)==handles.lights_OFF(1)        % Turn light OFF if light's ON and t > lightsOFF time
        if t(2)>=handles.lights_OFF(2)
            writeInfraredWhitePanel(handles.teensy_port,0,0);
            lightStatus=0;
            writeInfraredWhitePanel(handles.teensy_port,0,0);
        end
    elseif lightStatus==0 && t(1)==handles.lights_ON(1)             % Turn light ON if light's OFF and t > lightsON time
            if t(2)>=handles.lights_ON(2) && t(2)<handles.lights_OFF(2)
                writeInfraredWhitePanel(handles.teensy_port,0,handles.White_intensity);
                lightStatus=1;
            end
    end
    
    %% Update frame counter and timestamp
    if mod(counter,30)==0
       cla reset
       imagesc(im, 'Parent', handles.axes1);
       hold on
       % Mark centroids
       plot(lastCentroid(:,1),lastCentroid(:,2),'o','Color','r');
       hold off
       drawnow
    end
    
    set(handles.edit_frame_rate,'String',num2str(1/(tElapsed-prev_tStamp)));
    counter=counter+1;
    prev_tStamp=tElapsed;
    
    % Clear variables to keep memory available
    clearvars im diffIm props validCentroids cenDat speeds dist dt

end

t = clock;
tON = handles.lightsON;
tOFF = handles.lightsOFF;
m_freq=handles.pulse_frequency;
m_interval=handles.pulse_interval;
m_pulse_number=handles.pulse_number;
m_pulse_amp=handles.pulse_amplitude;
fpath=handles.fpath;

%% Process data
clearvars -except motorID cenID t tON tOFF m_freq m_interval m_pulse_number m_pulse_amp fpath strain treatment tStart

% Create a plot of the centroid data as a check on the tracking
cenDat=dlmread(cenID);
x=cenDat(mod(1:size(cenDat,1),2)==1,:);
y=cenDat(mod(1:size(cenDat,1),2)==0,:);
clearvars cenDat

% Subsample the data to 1,000 data points per fly
f=round(size(x,1)/1000);
figure(1);

for i=1:size(x,2)
    hold on
    plot(x(mod(1:size(x,1),f)==0,i),y(mod(1:size(y,1),f)==0,i));
    hold off
end

clearvars x y

% Create plots for circadian data
interval=2;         % Width of sliding window in min
stepSize=0.2;       % Incrimental step size of sliding window in min

motorData=dlmread(motorID);

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


%% Plot population mean over time and plot
meanPlot=nanmean(plotData,2);
meanPlot(1)=[];

plot(meanPlot,'r');
hold on
for i=1:length(mi2);
    plot([mi1(i) mi1(i)],[0 max(meanPlot)+1],'Color',[0 0 1],'Linewidth',2);
end

activityTrace = findobj(gca, 'Color', 'r');
uistack(activityTrace, 'top')

%%
plotData(1,:)=[];

circData.plotData=plotData;
circData.tElapsed=tElapsed;
circData.activeFlies=nanmean(speed)>0.01;
circData.numActive=sum(circData.activeFlies);
circData.speed=speed;
circData.motorON=motorON;
circData.motorOFF=motorOFF;
circData.lightsON=tON;
circData.lightsOFF=tOFF;
circData.experiment_end=t;
circData.freq=m_freq;
circData.amp=m_pulse_amp;
circData.interval=m_interval;
circData.nPulse=m_pulse_number;

save(strcat(tStart,strain,treatment,'circadian.mat'),circData);
