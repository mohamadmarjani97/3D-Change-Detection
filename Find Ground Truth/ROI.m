clc;
clear;
close all;
%% Read Data
ptCloud1=pcread('Final1.ply');

grid=5;
ptCloud1=pcdownsample(ptCloud1,'gridAverage',grid);

ptCloud2=pcread('Final2.ply');
ptCloud2=pcdownsample(ptCloud2,'gridAverage',grid);

%% Registration
[tform,~,rmse] = pcregistericp(ptCloud2,ptCloud1,'Extrapolate',true,'InlierRatio',.001);
movingReg = pctransform(ptCloud2,tform);

%% Extract Coordinate from Point clouds 5
X2=double(movingReg.Location(:,1));
Y2=double(movingReg.Location(:,2));
Z2=double(movingReg.Location(:,3));

X1=double(ptCloud1.Location(:,1));
Y1=double(ptCloud1.Location(:,2));
Z1=double(ptCloud1.Location(:,3));
points3D_1 = [X1 Y1 Z1];
points3D_2 = [X2 Y2 Z2];

pcshow(movingReg);
view(2);
%% roi Selection (use one from following)
roi = images.roi.Polygon(gca) ;
draw(roi)
pause(3)
%% select points inside roi
x = double(movingReg.Location(:,1));
y = double(movingReg.Location(:,2));
mask = inROI(roi,x,y);
G=find(mask==1);
sub_ptCloud = select(movingReg,G);

Xsub=double(sub_ptCloud.Location(:,1));
Ysub=double(sub_ptCloud.Location(:,2));
Zsub=double(sub_ptCloud.Location(:,3));

xlswrite('Test.xls',[Xsub,Ysub,Zsub]);

figure;
hold on
scatter3(Xsub,Ysub,Zsub,'o','b');
scatter3(X2,Y2,Z2,'.','r');
view(2);
