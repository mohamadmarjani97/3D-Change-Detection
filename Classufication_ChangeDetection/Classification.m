clc;
clear;
close all;

%% Read Data And Downsampling 
format long g
ptCloud1=pcread('Final1.ply');
ptCloud2=pcread('Final2.ply');
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
%% Merge X,Y And Z Point Cloud Data 
points3D_1 = [X1 Y1 Z1];
m1=size(points3D_1,1);
points3D_2 = [X2 Y2 Z2];
m2=size(points3D_2,1);

%% Classification
T1 = kmeans(Z1,5);
T2=kmeans(Z2,5);
Y=zeros(5,1);
for i=1:5
    B=find(T1==i);
    F=Z1(B);
    MeanZ1=mean(F);
    L=zeros(5,1);
    for j=1:5
        B1=find(T2==j);
        F1=Z2(B1);
        MeanZ2=mean(F1);
        L(j)=abs(MeanZ1-MeanZ2);
    end
    Q=find(L==min(L));
    Y(i)=Q;
end
    
for i=1:m1
    if T1(i)==1
        T1(i)=Y(1);
    elseif T1(i)==2
        T1(i)=Y(2);
    elseif T1(i)==3
        T1(i)=Y(3);
    elseif T1(i)==4
        T1(i)=Y(4);
    else
        T1(i)=Y(5);
    end
end


%% Tresholding
MinC=load('MinCoor.mat');
MinCoor=MinC.MinCoor;

Change=zeros(m2,1);
for i=1:m2
    i
    if MinCoor(i,2)<6
        
        if T2(i)~=T1(MinCoor(i,1))
            Change(i)=1;
        end
    else
        Change(i)=1;
    end
end

Change=find(Change==1);
P=points3D_2(Change,:);



figure;

subplot(2,2,1)
pcshow(points3D_1,T1)
colormap(hsv(5))
title('2016')

view(2)
subplot(2,2,2)
pcshow(points3D_2,T2)
colormap(hsv(5))
title('2018')

view(2)
subplot(2,2,[3,4])
pcshow(P);
title('Changed Points');
view(2);

%% Testing and Accuracy
ChangedPoints1=load('New_Ground_Original_Change.mat');
ChangedPoints=ChangedPoints1.New_Ground_Original_Change;

UnChangedPoints1=load('New_Unchanged.mat');
UnChangedPoints=UnChangedPoints1.New_Unchanged;


%%
ChangedNum=0;
for i=1:size(ChangedPoints,1)
    for j=1:size(P,1)
        if ChangedPoints(i,1)==P(j,1)
            if ChangedPoints(i,2)==P(j,2)
                if ChangedPoints(i,3)==P(j,3)
                    ChangedNum=ChangedNum+1;
                end
            end
        end
    end
end

%%
UnChangedNum=0;

for i=1:size(UnChangedPoints,1)
    for j=1:size(P,1)
        if UnChangedPoints(i,1)==P(j,1)
            if UnChangedPoints(i,2)==P(j,2)
                if UnChangedPoints(i,3)==P(j,3)
                    UnChangedNum=UnChangedNum+1;
                end
            end
        end
    end
end
        

%%
    
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


