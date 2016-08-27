function out=circPlotHandTraces(flyCircles,raw_data,centers,mode)

numFlies=length(flyCircles);
raw_data(:,1)=[];

colors = rand(1,3,numFlies);
numFigures=ceil(numFlies/10);
bins=0:2*pi/26:2*pi;
bins(end)=[];


for i=1:numFlies
   disp(i)
   if mod(i-1,10)==0
       figure()
       k=0;
   end
    subP=mod(i-1,5)+1+k*10;

    %Plot fly trace
    hold on
    subplot(5,5,subP);

            if mode
            xTrace=raw_data(flyCircles(i).valid_trials,i*2-1)-centers(i,1);
            yTrace=raw_data(flyCircles(i).valid_trials,i*2)-centers(i,2);
            tmpAngle=flyCircles(i).circum_vel;
            tmpAngle=tmpAngle(flyCircles(i).valid_trials);
            z=zeros(sum(flyCircles(i).valid_trials),1);
            mu=-sin(tmpAngle);
            surface([xTrace';xTrace'],[yTrace';yTrace'],[z';z'],[mu';mu'],...
                'facecol','no','edgecol','interp','linew',0.5);
            end


    % Plot angle histogram
    hold on
    subplot(5,5,subP+5);
    h1=plot(flyCircles(i).angleavg,'color',colors(:,:,i));
    xLabels={'0';'\pi/2';'\pi';'3\pi/2';'2\pi'};
    set(gca,'Xtick',[0 6 13.5 20 26],'XtickLabel',xLabels)
    set(h1,'Linewidth',2)
    legend(['u=' num2str(flyCircles(i).mu)],'Location','northeast')
    legend('boxoff')
    axis([0,length(bins),0,0.25]);
       if subP==5
            k=k+1;
       end 
       

    
end

end



