%% Start The program 
clc;
clear;
close all;

%% Read Data And Downsampling 
format long g
ptCloud1=pcread('Final1.ply');

grid=5;
ptCloud1=pcdownsample(ptCloud1,'gridAverage',grid);

ptCloud2=pcread('Final2.ply');
ptCloud2=pcdownsample(ptCloud2,'gridAverage',grid);

%% Register two Point Clouds 
[tform,~,rmse] = pcregistericp(ptCloud2,ptCloud1,'Extrapolate',true,'InlierRatio',.001);
movingReg = pctransform(ptCloud2,tform);

%% Show registration Result 

figure
pcshowpair(movingReg,ptCloud1,'MarkerSize',.005)
view(2)
xlabel('X')
xlabel('Y')
zlabel('Z')
title('Point clouds after registration')
legend('Moving point cloud','Fixed point cloud')
legend('Location','southoutside')


%% Extract Coordinate from Point clouds 
X2=double(movingReg.Location(:,1));
Y2=double(movingReg.Location(:,2));
Z2=double(movingReg.Location(:,3));

X1=double(ptCloud1.Location(:,1));
Y1=double(ptCloud1.Location(:,2));
Z1=double(ptCloud1.Location(:,3));

%% Merge X,Y And Z Point Cloud Data 
points3D_1 = [X1 Y1 Z1];
points3D_2 = [X2 Y2 Z2];


%% Create Octree Structure For Two Point Clouds Data 
OT1 = OcTree(points3D_1,'binCapacity',50);
OT2 = OcTree(points3D_2,'binCapacity',50);

%% Show Octree Results 
OT1.shrink
figure
boxH1 = OT1.plot;
cols1 = lines(OT1.BinCount);
doplot3_1 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
for i = 1:OT1.BinCount
    set(boxH1(i),'Color',cols1(i,:),'LineWidth', 1+OT1.BinDepths(i))
    doplot3_1(points3D_1(OT1.PointBins==i,:),'.','Color',cols1(i,:),'DisplayName',num2str(i))
end
axis image, view(3)
title('Octree 2016')
legend show



OT2.shrink
figure
boxH2 = OT2.plot;
cols2 = lines(OT2.BinCount);
doplot3_2 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
for i = 1:OT2.BinCount
    set(boxH2(i),'Color',cols2(i,:),'LineWidth', 1+OT2.BinDepths(i))
    doplot3_2(points3D_2(OT2.PointBins==i,:),'.','Color',cols2(i,:),'DisplayName',num2str(i))
end
axis image, view(3)
title('Octree 2018');
legend show




%% Fit A Plane to Each Bins of OT1 Points 
XPlane1=inf(50,50,OT1.BinCount);
YPlane1=inf(50,50,OT1.BinCount);
ZPlane1=inf(50,50,OT1.BinCount);
B1=Inf(3,OT1.BinCount);

for i=1:OT1.BinCount
    Coor1=points3D_1(OT1.PointBins==i,:);
    [m1,n1]=size(Coor1);
    NewX=Coor1(:,1);
    NewY=Coor1(:,2);
    NewZ=Coor1(:,3);
    clear Coor1
    if m1>=1
        [XPlane1(:,:,i),YPlane1(:,:,i),ZPlane1(:,:,i),B1(1:3,i)]=PlaneFitting(NewX,NewY,NewZ,50);
        i;
    end
end

%% Fit A Plane to Each Bins of OT2 Points 
XPlane2=inf(50,50,OT2.BinCount);
YPlane2=inf(50,50,OT2.BinCount);
ZPlane2=inf(50,50,OT2.BinCount);

B2=Inf(3,OT2.BinCount);
for i=1:OT2.BinCount
    Coor2=points3D_2(OT2.PointBins==i,:);
    [m2,n2]=size(Coor2);
    NewX=Coor2(:,1);
    NewY=Coor2(:,2);
    NewZ=Coor2(:,3);
    clear Coor2
    if m2>=1
        [XPlane2(:,:,i),YPlane2(:,:,i),ZPlane2(:,:,i),B2(1:3,i)]=PlaneFitting(NewX,NewY,NewZ,50);
        i;
    end
end


%% Match Each Bin of OT1 with OT2 Bins 
MatchingBin=BinMatching(OT1,points3D_1,OT2,points3D_2);


%% Calculate Angele Of Planes 
Angle_OT1_OT2=inf(OT2.BinCount,1);
for i=1:size(B2,2)
    
    if MatchingBin(i)~=inf
        
        a1=Angle_Between_Two_Plane(XPlane2(:,:,i),YPlane2(:,:,i),ZPlane2(:,:,i),XPlane1(:,:,MatchingBin(i)),YPlane1(:,:,MatchingBin(i)),ZPlane1(:,:,MatchingBin(i)));
        Angle_OT1_OT2(i,1)=a1;
    end
  
end


%% Treshholding For Extract Changed Points 
Angle_OT1_OT2=abs(Angle_OT1_OT2);

Q=find(isnan(Angle_OT1_OT2));
Q1=find(Angle_OT1_OT2>8 & Angle_OT1_OT2<3000);
w2=[Q;Q1];
U2=zeros(size(points3D_2,1),1);
for i=1:size(w2,1)
    i;
    
    for j=1:size(points3D_2,1)
        
        if w2(i)==OT2.PointBins(j)
            
            U2(j)=1;
        end
    end
end
CheangedPoints2=find(U2==1);
%% show Final Results 
figure;
subplot(2,2,1);
pcshow(ptCloud1);
title('2016')

subplot(2,2,2);
pcshow(ptCloud2);
title('2018');
subplot(2,2,[3,4]);
pcshow(ptCloud2.Location(CheangedPoints2(1:end),1:3))
title('Changed Points')


%% Testing And Accuracy
ChangedPoints=load('New_DownSample_Ground_Change.mat');
Change=ChangedPoints.New_DownSample_Ground_Change;

UnChangedPoints=load('New_Unchanged_DownSample.mat');
Unchanged=UnChangedPoints.New_Unchanged_DownSample;

Data(:,1)=X2(CheangedPoints2,1);
Data(:,2)=Y2(CheangedPoints2,1);
Data(:,3)=Z2(CheangedPoints2,1);

ChangedNum=0;
for i=1:1113
    for j=1:1727
        if Change(i,1)==Data(j,1)
            if Change(i,2)==Data(j,2)
                if Change(i,3)==Data(j,3)
                    ChangedNum=ChangedNum+1;
                end
            end
        end
    end
end
UnChangedNum=0;
for i=1:2074
    for j=1:1727
        if Unchanged(i,1)==Data(j,1)
            if Unchanged(i,2)==Data(j,2)
                if Unchanged(i,3)==Data(j,3)
                    UnChangedNum=UnChangedNum+1;
                end
            end
        end
    end
end

Conf(1,1)=ChangedNum;
Conf(1,2)=size(Change,1)-ChangedNum;
Conf(2,2)=size(Unchanged,1)-UnChangedNum;
Conf(2,1)=UnChangedNum;

%% Kappa Coefficient
M=sum(sum(Conf));

Kapa=(M*sum(diag(Conf))-(sum(Conf(1,:))*sum(Conf(:,1))+sum(Conf(2,:))*sum(Conf(:,2))))/(M^2-(sum(Conf(1,:))*sum(Conf(:,1))+sum(Conf(2,:))*sum(Conf(:,2))))*100;


%% Overall Accuracy
OverallAccuracy=(sum(diag(Conf))/M)*100;

%% Producers's Accuray= Class Accuracy
Change_Producers_Accuracy=(Conf(1,1)/sum(Conf(1,:)))*100;
No_Change_Producers_Accuracy=(Conf(2,2)/sum(Conf(2,:)))*100;

%% User's Accuracy
Change_Users_Accuracy=(Conf(1,1)/sum(Conf(:,1)))*100;
No_Change_Users_Accuracy=(Conf(2,2)/sum(Conf(:,2)))*100;

%% Omission Error
Change_Omission_Error=100-Change_Producers_Accuracy;
No_Change_Omission_Error=100-No_Change_Producers_Accuracy;
%% Comission Error
Change_Comission_Error=100-Change_Users_Accuracy;
No_Change_Comission_Error=100-No_Change_Users_Accuracy;

%%
TP=Conf(1,1);
FP=Conf(2,1);
TN=Conf(2,2);
FN=Conf(1,2);

Completeness=TP/(TP+FN);
Corectness=TP/(TP+FP);
Quality=TP/(TP+FN+FP);

Accuracy={'Producers Accuracy';'Users Accuracy';'Omission Error';'Commission Error'};
Change=[Change_Producers_Accuracy;Change_Users_Accuracy;Change_Omission_Error;Change_Comission_Error];
NoChange=[No_Change_Producers_Accuracy;No_Change_Users_Accuracy;No_Change_Omission_Error;No_Change_Comission_Error];
T=table(Accuracy,Change,NoChange)
