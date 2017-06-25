function  X_out  = TrackletAssociation(X,Ff)

X_org=X;
Time=Ff;
time1=zeros(1,2);
time2=zeros(1,2);
tracklet=size(X_org,2);
X_out=cell(1,tracklet);
X_out1=cell(1,tracklet);
jointset=zeros(tracklet,tracklet);
flage=zeros(tracklet,tracklet);
dis=zeros(tracklet,tracklet);
H=[1 0 0 0;0 0 1 0];
dis_non=0;
J1=zeros(tracklet,tracklet);
% num_d=0;
for i=1:size(X_org,2)
    time1=Time{1,i}(:,[1 end]);
    x1_start=H*X_org{1,i}(:,1);
    x1_end=H*X_org{1,i}(:,end);
    for j=1:size(X_org,2)
        time2=Time{1,j}(:,[1 end]);
        x2_start=H*X_org{1,j}(:,1);
        x2_end=H*X_org{1,j}(:,end);
        if(time1(2)<time2(1)||time1(1)>time2(2))
            gap=min(abs(time1(2)-time2(1)),abs(time1(1)-time2(2)));
            flage(i,j)=1;
            if(gap<10)
               dis1=((x1_start(1,:)-x2_end(1,:))^2+(x1_start(2,:)-x2_end(2,:))^2) ^0.5;
               dis2=((x1_end(1,:)-x2_start(1,:))^2+(x1_end(2,:)-x2_start(2,:))^2)^0.5;
               dis(i,j)=min(dis1,dis2);
               dis_non=dis_non+dis(i,j);
%                num_d=num_d+1;
            else
                dis(i,j)=inf;
            end
        else
            dis(i,j)=inf;
        end
    end
    dis(i,i)=max(0,dis_non);      
   dis_min=min(min(dis(i,:)));
    if(dis_min<20)
        jointlet=find(dis(i,:)==dis_min);
        if(jointlet==i)
          X_out1{1,i}=X_org{1,i};
        else
         time2=Time{1,jointlet}(:,[1 end]);
         X_out1{1,i}(:,time1(:,1):time1(:,2))=X_org{1,i}(:,:);
         X_out1{1,i}(:,time2(:,1):time2(:,2))=X_org{1,jointlet}(:,:);
         jointset(i,jointlet)=1;
        end
    else
        X_out1{1,i}=X_org{1,i};
    end
    
end
for i=1:16
    J=find(jointset(i,:)==1);
    if(~J)
    X_out{1,ii}=X_out1{1,ii};
    elseif
        X_out{1,ii}=X_out1{i,J};
        J1(i,J)=i;
end
end
end

