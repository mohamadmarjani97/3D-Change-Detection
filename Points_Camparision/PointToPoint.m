clc;
clear;
close all;

%% Read Data And Downsampling 
format long g
ptCloud1=pcread('Final1.ply');

% grid=5;
% ptCloud1=pcdownsample(ptCloud1,'gridAverage',grid);

ptCloud2=pcread('Final2.ply');
% ptCloud2=pcdownsample(ptCloud2,'gridAverage',grid);

%% Register two Point Clouds 
[tform,~,rmse] = pcregistericp(ptCloud2,ptCloud1,'Extrapolate',true,'InlierRatio',.001);
movingReg = pctransform(ptCloud2,tform);

%% Extract Coordinate from Point clouds 
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
Dis=zeros(m1,1);
MinCoor=zeros(m2,2);
for i=1:m2
    i
    for j=1:m1
        Dis(j)=sqrt((X1(j)-X2(i))^2+(Y1(j)-Y2(i))^2+(Z1(j)-Z2(i))^2);
    end
    MinCoor(i,1)=find(Dis==min(Dis));
    MinCoor(i,2)=Dis(MinCoor(i,1));
end
% M=load('MinCoor.mat');
% MinCoor=M.MinCoor;

%% Tresholding      
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

%% Testing And Accuracy
ChangedPoints1=load('New_Ground_Original_Change.mat');
ChangedPoints=ChangedPoints1.New_Ground_Original_Change;

UnChangedPoints1=load('New_Unchanged.mat');
UnChangedPoints=UnChangedPoints1.New_Unchanged;


ChangedNum=0;
for i=1:9828
    for j=1:22194
        if ChangedPoints(i,1)==E(j,1)
            if ChangedPoints(i,2)==E(j,2)
                if ChangedPoints(i,3)==E(j,3)
                    ChangedNum=ChangedNum+1;
                end
            end
        end
    end
end


UnChangedNum=0;

for i=1:26505
    for j=1:22194
        if UnChangedPoints(i,1)==E(j,1)
            if UnChangedPoints(i,2)==E(j,2)
                if UnChangedPoints(i,3)==E(j,3)
                    UnChangedNum=UnChangedNum+1;
                end
            end
        end
    end
end
        


    
Conf(1,1)=ChangedNum;
Conf(1,2)=size(ChangedPoints,1)-ChangedNum;
Conf(2,2)=size(UnChangedPoints,1)-UnChangedNum;
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


