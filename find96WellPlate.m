function out=find96WellPlate(im,plotBool,xPlate,yPlate)

thrVec=linspace(175,225,10);
erodeDiam=2;

wellLeftStart=0.06;
wellRightEnd=0.94;
wellYCorrection=0.97;

targetSize=(xPlate(1)-xPlate(4))*(yPlate(2)-yPlate(1));
targetSizeVec=zeros(length(thrVec),1);
for i=1:length(thrVec)
    regions=regionprops(im<thrVec(i),'Area');
    regAreaVec=[regions.Area];
    regAreaVec=sort(regAreaVec);
    if length(regAreaVec) >1
        regAreaVec=regAreaVec(end-1);
        targetSizeVec(i)=abs(regAreaVec-targetSize);
    else
        targetSizeVec(i)=Inf;
    end
    
end

whichThr=find(targetSizeVec==min(targetSizeVec));
whichThr=whichThr(1);
bestThr=thrVec(whichThr);
imThr=(im<bestThr);
regions=regionprops(imThr,'Area','Centroid','Extrema','PixelList');
regAreaVec=[regions.Area];
regAreaError=abs(regAreaVec-targetSize);
which=find(regAreaError==min(regAreaError));
which=which(1);
targetPixels=regions(which).PixelList;

%Gets coordinates of the plate
UL=[xPlate(1) yPlate(1)];
UR=[xPlate(2) yPlate(2)];
LR=[xPlate(3) yPlate(3)];

edgeAngle=atan2((UL(2)-UR(2)),(UL(1)-UR(1)));
edgeLength=sqrt((UR(2)-UL(2))^2+(UR(1)-UL(1))^2);
MP=[(UR(1)+UL(1))/2 (UR(2)+UL(2))/2];
longEdgeLength=sqrt((LR(2)-UR(2))^2+(LR(1)-UR(1))^2);

wellColCoords=linspace(wellLeftStart,wellRightEnd,12);
%{
wellColCoords=wellColCoords(7:12);
wellColCoords=wellColCoords-0.5;
wellColCoords=wellColCoords.^1.1;
wellColCoords=[flip(-wellColCoords) wellColCoords];
wellColCoords=wellColCoords+0.5;
wellColCoords-linspace(wellLeftStart,wellRightEnd,12);
%}

wellAngle=edgeAngle-pi()/2;
wellColXVec=MP(1)+longEdgeLength*cos(wellAngle)*wellColCoords;
x_center=mean(wellColXVec);
wellColXVec=wellColXVec-x_center;
wellColXVec=wellColXVec(1:6);
wellColXVec=wellColXVec.^0.995;
wellColXVec=[wellColXVec flip(-wellColXVec)] + x_center;

wellColYVec=MP(2)+longEdgeLength*sin(wellAngle)*wellColCoords;
colScale=8/12*(wellRightEnd-wellLeftStart)*longEdgeLength*wellYCorrection;
wellRowCoords=linspace(colScale/2,-colScale/2,8);
wellRowCoords=wellRowCoords(1:4);
wellRowCoords=wellRowCoords.^(0.99);
wellRowCoords=flip([flip(-wellRowCoords) wellRowCoords]);

wellCoordinates=zeros(96,2);
for i=1:12
    wellXTemp=wellColXVec(i)+cos(edgeAngle)*wellRowCoords;
    wellYTemp=wellColYVec(i)+sin(edgeAngle)*wellRowCoords;
    wellCoordinates((i-1)*8+1:(i-1)*8+8,1)=wellXTemp';
    wellCoordinates((i-1)*8+1:(i-1)*8+8,2)=wellYTemp';
end
%{
if plotBool
    set(0,'defaultaxesposition',[0 0 1 1])
    image(im);
    colormap(repmat(linspace(0,1,256)',1,3));
    hold on;
    scatter(UR(1),UR(2),'ro')
    scatter(UL(1),UL(2),'ko')
    scatter(LR(1),LR(2),'wo')
    plot([UR(1) UR(1)+cos(edgeAngle)*edgeLength],[UR(2) UR(2)+sin(edgeAngle)*edgeLength],'k-');
    scatter(MP(1),MP(2),'k.')
    plot([MP(1) MP(1)+cos(edgeAngle-pi()/2)*longEdgeLength],[MP(2) MP(2)+sin(edgeAngle-pi()/2)*longEdgeLength],'k-');
    scatter(wellCoordinates(:,1),wellCoordinates(:,2),'r.')
    hold off;
    F=getframe(gcf);
    [X, ~] = frame2im(F);
    out.wellImage=X;
end
%}
out.corners=[UL LR];
out.coords=wellCoordinates;
out.colScale=colScale;
out.threshold=bestThr;

