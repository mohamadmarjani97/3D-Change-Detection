clc;
clear;
close all;

%% Read Data And Downsampling 2
format long g
ptCloud1=pcread('reconstructed_Output1.ply');
pcshow(ptCloud1)
view(2)

X=ptCloud1.Location(:,1:3);
T=kmeans(X(:,3),3);
pcshow(X,T)
colormap(hsv(3))
title('2016')
view(2)




