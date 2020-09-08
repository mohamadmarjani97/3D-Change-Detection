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




% CDMatrix=inf(OT2.BinCount,2);
%% Calculate std of each OT2
RE2=inf(OT2.BinCount,1);
for i=1:OT2.BinCount
    Coor2=points3D_2(OT2.PointBins==i,:);
    [m2,n2]=size(Coor2);
    NewX=Coor2(:,1);
    NewY=Coor2(:,2);
    NewZ=Coor2(:,3);
    clear Coor2
    if m2>=1
        X_std2=std(NewX);
        Y_std2=std(NewY);
        Z_std2=std(NewZ);
        
        RE2(i)=sqrt(X_std2^2+Y_std2^2+Z_std2^2);
    end
end

%% Calculate std of each OT1
RE1=inf(OT1.BinCount,1);
for i=1:OT1.BinCount
    Coor1=points3D_1(OT1.PointBins==i,:);
    [m1,n1]=size(Coor1);
    NewX=Coor1(:,1);
    NewY=Coor1(:,2);
    NewZ=Coor1(:,3);
    clear Coor1
    if m1>=1
        X_std1=std(NewX);
        Y_std1=std(NewY);
        Z_std1=std(NewZ);
        
        RE1(i)=sqrt(X_std1^2+Y_std1^2+Z_std1^2);
    end
end

%% Matching Bin
BinMatching=inf(OT2.BinCount,1);
for i=1:OT2.BinCount
    if RE2(i)<20
        i
        Coor2=points3D_2(OT2.PointBins==i,:);
        [m2,n2]=size(Coor2);
        NewX=mean(Coor2(:,1));
        NewY=mean(Coor2(:,2));
        NewZ=mean(Coor2(:,3));
        
        
        
        
        Radius=zeros(m2,1);
        for w=1:m2
            Radius(w)=sqrt((Coor2(w,1)-NewX)^2+(Coor2(w,2)-NewY)^2+(Coor2(w,3)-NewZ));
        end
        Radius=max(Radius);
        P=zeros(OT1.BinCount,1);
        for j=1:OT1.BinCount
            Coor1=points3D_1(OT1.PointBins==j,:);
            NewX1=Coor1(:,1);
            NewY1=Coor1(:,2);
            NewZ1=Coor1(:,3);
            [m1,n1]=size(Coor1);
            if m1>0
                
                Dis=zeros(m1,2);
                for i1=1:m1
                    dx=NewX-NewX1(i1);
                    dy=NewY-NewY1(i1);
                    dz=NewZ-NewZ1(i1);
                    Dis(i1,1)=sqrt(dx^2+dy^2+dz^2);
                    if Dis(i1,1)>Radius
                        Dis(i1,2)=0;
                    else
                        Dis(i1,2)=1;
                    end
                end
                Loc_NA=find(Dis(:,2)==1);
                NA=size(Loc_NA,1);
                NS=m1;
                P(j)=NA/NS;
            end
        end
        if norm(P)>0
            tr=find(P>.5);
            Size_tr=size(tr,1);
            if Size_tr==1
                Coor1tr=points3D_1(OT1.PointBins==tr,:);
                if m1>size(Coor1tr,1)-7 && m1<size(Coor1tr,1)+7
                    BinMatching(i)=tr;
                end
            end
            if Size_tr>1
                
                FD=zeros(Size_tr,1);
                for i2=1:Size_tr
                    Coor1tr=points3D_1(OT1.PointBins==tr(i2),:);
                    FD(i2)=size(Coor1tr,1);
                end
                DiffSize=abs(FD-m2);
                GH=find(DiffSize==min(DiffSize));
                if size(GH,1)>1
                    Diff=inf(size(GH,1),1);
                    for i3=1:size(GH,1)
                        Coor=points3D_1(OT1.PointBins==tr(GH(i3)),:);
                        Mean_CoorX=mean(Coor(:,1));
                        Mean_CoorY=mean(Coor(:,2));
                        Mean_CoorZ=mean(Coor(:,3));
                        Diff(i3)=(Mean_CoorX-NewX)^2+(Mean_CoorY-NewY)^2+(Mean_CoorZ-NewZ)^2;
                    end
                    K=find(Diff==min(Diff));
                    BinMatching(i)=tr(GH(K));
                else
                    BinMatching(i)=tr(GH);
                end
            end
        end
    end
end

for i=1:OT2.BinCount
    Coor2=points3D_2(OT2.PointBins==i,:);
    [m2,n2]=size(Coor2);
    NewX=mean(Coor2(:,1));
    NewY=mean(Coor2(:,2));
    NewZ=mean(Coor2(:,3));
    
    if m2==0
        BinMatching(i)=0;
    end
end
%
%
for i=1:OT2.BinCount
    
    
    if RE2(i)>20 && RE2(i)<10000
        BinMatching(i)=0.5;
    end
end

A=[1,1,1];
for i=1:size(BinMatching,1)
    i
    if BinMatching(i)==inf
        Coor2=points3D_2(OT2.PointBins==i,:);
        [m2,n2]=size(Coor2);
        NewX=Coor2(:,1);
        NewY=Coor2(:,2);
        NewZ=Coor2(:,3);
        
        subplot(2,2,1)
        pcshow(points3D_1);
        title('2016');
        view(2)

        subplot(2,2,2)
        pcshow(points3D_2)
        title('2018');
        view(2);
        subplot(2,2,[3,4])
        hold on
        pcshow(Coor2)
        title('Changed Points');
        view(2);
        A=cat(1,A,Coor2)
        
    end
end

%% Testing And Accuracy
A(1,:)=[];
ChangedPoints=load('New_DownSample_Ground_Change.mat');
Change=ChangedPoints.New_DownSample_Ground_Change;

UnChangedPoints=load('New_Unchanged_DownSample.mat');
Unchanged=UnChangedPoints.New_Unchanged_DownSample;


Data=A;


ChangedNum=0;
for i=1:size(Change,1)
    for j=1:size(A,1)
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

for i=1:size(Unchanged,1)
    for j=1:size(Data,1)
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