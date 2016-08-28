function out=flyBurHandData_legacy(centroid,numFlies,centers)

%flyBurHandData(data,roi)
bins=linspace(0,50,200);
out={};

for j=1:numFlies
    inx=centroid(:,2*j-1);
    iny=centroid(:,2*j);
    
    out(j).x=inx;
    out(j).y=iny;
    out(j).r=sqrt((inx-centers(j,1)).^2+(iny-centers(j,2)).^2);
    out(j).theta=atan2(iny-centers(j,2),inx-centers(j,1));
    dhist=histc(out(j).r,bins);
    dhist=dhist./sum(dhist);
    dhistCDF=cumsum(dhist);
    [v,g]=min(abs(0.95-dhistCDF));
    out(j).width=bins(g);
    
    out(j).direction=zeros(size(inx,1),1);
    out(j).speed=zeros(size(inx,1),1);
    out(j).turning=zeros(size(inx,1),1);

    iny_shifted=[iny(2:end);iny(end)];
    inx_shifted=[inx(2:end);inx(end)];    

    out(j).direction=atan2(iny_shifted-iny,inx_shifted-inx);
    out(j).direction=[NaN;out(j).direction];
    out(j).direction(end)=[];
    out(j).speed=sqrt((iny_shifted-iny).^2+(inx_shifted-inx).^2);
    out(j).speed=[NaN;out(j).speed];
    out(j).speed(end)=[];
    out(j).turning=out(j).direction(2:end)-out(j).direction(1:length(out(j).direction)-1);
    out(j).turning=[NaN;out(j).turning];
    out(j).turning(out(j).turning>pi)=out(j).turning(out(j).turning>pi)-2*pi;
    out(j).turning(out(j).turning<-pi)=out(j).turning(out(j).turning<-pi)+2*pi;

end

