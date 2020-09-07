clc;
clear;
close all;

%% Read Data And Downsampling 2
format long g
ptCloud1=pcread('Final1.ply');

% grid=5;
% ptCloud1=pcdownsample(ptCloud1,'gridAverage',grid);

ptCloud2=pcread('Final2.ply');
% ptCloud2=pcdownsample(ptCloud2,'gridAverage',grid);

%% Register two Point Clouds 3
[tform,~,rmse] = pcregistericp(ptCloud2,ptCloud1,'Extrapolate',true,'InlierRatio',.001);
movingReg = pctransform(ptCloud2,tform);

%% Extract Coordinate from Point clouds 5
X2=double(movingReg.Location(:,1));
Y2=double(movingReg.Location(:,2));
Z2=double(movingReg.Location(:,3));

X1=double(ptCloud1.Location(:,1));
Y1=double(ptCloud1.Location(:,2));
Z1=double(ptCloud1.Location(:,3));


%% Merge X,Y And Z Point Cloud Data 6
points3D_1 = [X1 Y1 Z1];
m1=size(points3D_1,1);
points3D_2 = [X2 Y2 Z2];
m2=size(points3D_2,1);


%% Point Matching
% Dis=zeros(m1,1);
% MinCoor=zeros(m2,2);
% for i=1:m2
%     i
%     for j=1:m1
%         Dis(j)=sqrt((X1(j)-X2(i))^2+(Y1(j)-Y2(i))^2+(Z1(j)-Z2(i))^2);
%     end
%     MinCoor(i,1)=find(Dis==min(Dis));
%     MinCoor(i,2)=Dis(MinCoor(i,1));
% end
M=load('MinCoor.mat');
MinCoor=M.MinCoor;
        
C=MinCoor(:,2)>3;
I=find(C==1);
E=points3D_2(C,1:3);


figure;

subplot(2,2,1)
pcshow(points3D_1);
title('2016');
view(2)

subplot(2,2,2)
pcshow(points3D_2)
title('2018');
view(2);

subplot(2,2,[3,4])
pcshow(E)
title('Changed Points')
view(2)


