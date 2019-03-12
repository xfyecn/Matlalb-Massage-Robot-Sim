clear;
clc;
% Img = imread('massage_picture.jpg');
% level=graythresh(Img);
% q1=length(Img(:,1,1));
% q2=length(Img(1,:,1));
% x=q1:-1:1;
% y=1:q2;
% [X,Y]=meshgrid(x,y);
% figure(1);
% plot3(X,Y,Img(:,:,1),'k');
% figure(2);%image perform in figure
% MassageImg = imresize(im2bw(Img,0.95),0.3);
% MassageImg = ~MassageImg;
% MassageImg = flipud(MassageImg);
% [col, row] = size(MassageImg);
% imshow(MassageImg);
%%
p=imread('massage_picture.jpg');
g=rgb2gray(p); % 转为灰阶图
gg=double(g); % 转为数值矩阵
gg=1-gg/255; % 将彩色值转为 0-1 的渐变值
[x,y]=size(gg); % 取原图大小
[X,Y]=meshgrid(1:y,1:x); % 以原图大小构建网格
n=0.2;
mesh(-n*X,-n*Y,-gg); % 网格上画出图像
colormap gray; % 设为灰阶图像