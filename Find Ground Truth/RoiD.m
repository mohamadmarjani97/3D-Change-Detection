clc;
clear;
close all;

%% 
Ground1=xlsread('Change1.xls');
Ground2=xlsread('Change2.xls');
Ground3=xlsread('Change3.xls');
Ground4=xlsread('Change4.xls');
Ground5=xlsread('Change5.xls');
Ground6=xlsread('Change6.xls');
Ground7=xlsread('Change7.xls');
Ground8=xlsread('Change8.xls');
Ground9=xlsread('Change9.xls');
Ground10=xlsread('Change10.xls');
Ground11=xlsread('Change11.xls');
Ground12=xlsread('Change12.xls');

%%
changed_Down1=xlsread('changed_Down1.xls');
changed_Down2=xlsread('changed_Down2.xls');
changed_Down3=xlsread('changed_Down3.xls');
changed_Down4=xlsread('changed_Down4.xls');
changed_Down5=xlsread('changed_Down5.xls');
changed_Down6=xlsread('changed_Down6.xls');
changed_Down7=xlsread('changed_Down7.xls');
changed_Down8=xlsread('changed_Down8.xls');
changed_Down9=xlsread('changed_Down9.xls');
changed_Down10=xlsread('changed_Down10.xls');
changed_Down11=xlsread('changed_Down11.xls');


%%
Unchange_DownSample1=xlsread('Unchanged_Down1.xls');
Unchange_DownSample2=xlsread('Unchanged_Down2.xls');
Unchange_DownSample3=xlsread('Unchanged_Down3.xls');
Unchange_DownSample4=xlsread('Unchanged_Down4.xls');
Unchange_DownSample5=xlsread('Unchanged_Down5.xls');

%%
Unchange1=xlsread('Unchanged1.xls');
Unchange2=xlsread('Unchanged2.xls');
Unchange3=xlsread('Unchanged3.xls');
Unchange4=xlsread('Unchanged4.xls');
Unchange5=xlsread('Unchanged5.xls');
Unchange6=xlsread('Unchanged6.xls');
Unchange7=xlsread('Unchanged7.xls');
Unchange8=xlsread('Unchanged8.xls');


%%
Ground_Orginal=[Ground1;Ground2;Ground3;Ground4;Ground5;Ground6;Ground7;Ground8;Ground9;Ground10;Ground11;Ground12];
New_Ground_Original_Change=unique(Ground_Orginal,'rows');
%%

DownSample_Ground=[changed_Down1;changed_Down2;changed_Down3;changed_Down4;changed_Down5;changed_Down6;changed_Down7;changed_Down8;changed_Down9;changed_Down10;changed_Down11];
New_DownSample_Ground_Change=unique(DownSample_Ground,'rows');
%%
Unchanged=[Unchange1;Unchange2;Unchange3;Unchange4;Unchange5;Unchange6;Unchange7;Unchange8];
New_Unchanged=unique(Unchanged,'rows')
%%
Unchange_DownSample=[Unchange_DownSample1;Unchange_DownSample2;Unchange_DownSample3;Unchange_DownSample4;Unchange_DownSample5];
New_Unchanged_DownSample=unique(Unchange_DownSample,'rows');
