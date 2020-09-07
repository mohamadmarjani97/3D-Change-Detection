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
