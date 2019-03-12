close all;
clear;
%%
%%%%%%%%%%Put picture on XYZ%%%%%%%%%%%%%%%
% Img = imread('massage_picture.jpg');
% level=graythresh(Img);
% q1=length(Img(:,1,1));
% q2=length(Img(1,:,1));
% x=q1:-1:1;
% y=1:q2;
% [X,Y]=meshgrid(x,y);
% figure(1);
% plot3(X,Y,Img(:,:,1),'k');hold on;
p=imread('massage_picture.jpg');
% p=imrotate(p,-90);
g=rgb2gray(p); % 转为灰阶图
gg=double(g); % 转为数值矩阵
gg=1-gg/255; % 将彩色值转为 0-1 的渐变值
[x,y]=size(gg); % 取原图大小
[X,Y]=meshgrid(1:y,1:x); % 以原图大小构建网格
n=0.01;
x_tran=-1.5;%-:向左 +:向右
y_tran=0;%+:向上 -:向下
z_tran=6;%UR5末端XY坐标和背部中心重合
% z_tran=5;%Massage_robot末端XY坐标和背部中心重合
% mesh(-n*X+x_tran,-n*Y+y_tran,-gg-z_tran); % 网格上画出图像
axis([-20,20,-20,20,-15,15]);
% colormap gray; % 设为灰阶图像
% hold on;
% figure(2);%image perform in figure
% MassageImg = imresize(im2bw(Img,0.95),0.3);
% MassageImg = ~MassageImg;
% MassageImg = flipud(MassageImg);
% [col, row] = size(MassageImg);
% imshow(MassageImg);hold on;
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UR5_initial
startup_rvc;%launch robotics toolbox
global robot 
UR5mdl;
% parameter in robotics toolbox
th1=0;th2=0;th3=0;th4=0;th5=0;th6=0;
move_one_step(th1,th2,th3,th4,th5,th6,0);%initial
% pause;
% Z_label=-6;
% pL=[1 0 0 -5;0 -1 0 1.5;0 0 -1 Z_label;0 0 0 1];
% q1=robot.ikine(pL);
% th1=q1(1);th2=q1(2);th3=q1(3);th4=q1(4);th5=q1(5);th6=q1(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Massage_robot_initial
% startup_rvc;%launch robotics toolbox
% global robot 
% Massage_robot4DOF;
% % parameter in robotics toolbox
% th1=pi/2;th2=0;th3=0;th4=0;th5=0;th6=0;
% move_one_step(th1,th2,th3,th4,th5,th6,0);%initial
% % pause;
%%
[row,col]=size(gg);z_label=-6;ii=1;
row=-n*row+y_tran;col=-n*col+x_tran;index_minx=-5.7;index_maxx=-3.7;
for y_label=y_tran+2:-0.2:row+2
    for x_label=-2.7:-0.2:-6.7
        if index_minx<x_label&&x_label<index_maxx
              for zz_label=z_label:-0.2:-6.5
                  pL=[1 0 0 x_label;0 -1 0 y_label;0 0 -1 zz_label;0 0 0 1];
                  q=robot.ikine(pL);  
                  q1(ii,:)=q;
                  th1=q(1);th2=q(2);th3=q(3);th4=q(4);th5=q(5);th6=q(6);
                  move_one_step(th1,th2,th3,th4,th5,th6,0);
                  robotxyz=robot.fkine([th1,th2,th3,th4,th5,th6]);
                  plot3(robotxyz.t(1),robotxyz.t(2),robotxyz.t(3),'ro');hold on;
                  ii=ii+1;
              end                         
        else
            pL=[1 0 0 x_label;0 -1 0 y_label;0 0 -1 z_label;0 0 0 1];
            q=robot.ikine(pL);  
            q1(ii,:)=q;
            th1=q(1);th2=q(2);th3=q(3);th4=q(4);th5=q(5);th6=q(6);
            move_one_step(th1,th2,th3,th4,th5,th6,0);
            robotxyz=robot.fkine([th1,th2,th3,th4,th5,th6]);
            plot3(robotxyz.t(1),robotxyz.t(2),robotxyz.t(3),'ro');hold on;
            ii=ii+1;
        end
    end
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UR5
%扫描图片得到坐标信息，逆运动学求解机器人工作位姿
% Z_label=-7;
% pL=[1 0 0 -0.5;0 -1 0 -3.74;0 0 -1 Z_label;0 0 0 1];
% q1=robot.ikine(pL);
% th1=q1(1);th2=q1(2);th3=q1(3);th4=q1(4);th5=q1(5);th6=q1(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
% % pause;
% pR=[1 0 0 -6.54;0 -1 0 -3.78;0 0 -1 Z_label;0 0 0 1];
% q2=robot.ikine(pR);
% th1=q2(1);th2=q2(2);th3=q2(3);th4=q2(4);th5=q2(5);th6=q2(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
% % pause;
% 
% pU1=[1 0 0 -2.13;0 -1 0 -1.08;0 0 -1 Z_label;0 0 0 1];
% q3=robot.ikine(pU1);
% th1=q3(1);th2=q3(2);th3=q3(3);th4=q3(4);th5=q3(5);th6=q3(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
% % pause;
% % 
% pU2=[1 0 0 -5.53;0 -1 0 -0.66;0 0 -1 Z_label;0 0 0 1];
% q4=robot.ikine(pU2);
% th1=q4(1);th2=q4(2);th3=q4(3);th4=q4(4);th5=q4(5);th6=q4(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
% % pause;
% % 
% pD1=[1 0 0 -2.19;0 -1 0 -5.87;0 0 -1 Z_label;0 0 0 1];
% q6=robot.ikine(pD1);
% th1=q6(1);th2=q6(2);th3=q6(3);th4=q6(4);th5=q6(5);th6=q6(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
% % % pause;
% pD2=[1 0 0 -5.0;0 -1 0 -5.63;0 0 -1 Z_label;0 0 0 1];
% q5=robot.ikine(pD2);
% th1=q5(1);th2=q5(2);th3=q5(3);th4=q5(4);th5=q5(5);th6=q5(6);
% move_one_step(th1,th2,th3,th4,th5,th6,0);
% % % pause;
%%
% %图像坐标描点%根据个人定制
% [row,col]=size(gg);z_label=-5;ii=1;
% row=-n*row+y_tran;col=-n*col+x_tran;index_minx=-5.7;index_maxx=-3.7;
% for y_label=y_tran:-0.1:row
%     for x_label=-2.7:-0.1:-6.7
%         if index_minx<x_label&&x_label<index_maxx
%               for zz_label=z_label:-0.1:-5.5
%                   pL=[1 0 0 x_label;0 -1 0 y_label;0 0 -1 zz_label;0 0 0 1];
%                   q=robot.ikine(pL);  
%                   q1(ii,:)=q;
%                   th1=q(1);th2=q(2);th3=q(3);th4=q(4);th5=q(5);th6=q(6);
%                   move_one_step(th1,th2,th3,th4,th5,th6,0);
%                   robotxyz=robot.fkine([th1,th2,th3,th4,th5,th6]);
%                   plot3(robotxyz.t(1),robotxyz.t(2),robotxyz.t(3),'r.');hold on;
%                   ii=ii+1;
%               end                         
%         else
%             pL=[1 0 0 x_label;0 -1 0 y_label;0 0 -1 z_label;0 0 0 1];
%             q=robot.ikine(pL);  
%             q1(ii,:)=q;
%             th1=q(1);th2=q(2);th3=q(3);th4=q(4);th5=q(5);th6=q(6);
%             move_one_step(th1,th2,th3,th4,th5,th6,0);
%             robotxyz=robot.fkine([th1,th2,th3,th4,th5,th6]);
%             plot3(robotxyz.t(1),robotxyz.t(2),robotxyz.t(3),'r.');hold on;
%             ii=ii+1;
%         end
%     end
% end


% [Max_Q,Max_index]=max(q1,[],1);%the maximum value of q
% [Min_Q,Min_index]=min(q1,[],1);%the minimum value of q
% %通过限制每个关节最大最小角度规定UR5工作空间
% theta1=unifrnd(Min_Q(1),Max_Q(1),[1,30000]);%第一关节变量限位
% theta2=unifrnd(Min_Q(2),Max_Q(2),[1,30000]);%第二关节变量限位
% theta3=unifrnd(Min_Q(3),Max_Q(3),[1,30000]);%第三关节变量限位
% theta4=unifrnd(Min_Q(4),Max_Q(4),[1,30000]);%第四关节变量限位
% theta5=unifrnd(Min_Q(5),Max_Q(5),[1,30000]);%第五关节变量限位
% theta6=unifrnd(Min_Q(6),Max_Q(6),[1,30000]);%第六关节变量限位
% G = cell(30000, 6);%建立元胞数组
% for n = 1:30000
%     G{n} =[theta1(n) theta2(n) theta3(n) theta4(n) theta5(n) theta6(n)];
% end%产生30000组随机点
% H1=cell2mat(G);%将元胞数组转化为矩阵
% T=double(robot.fkine(H1));%机械臂正解
% scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)))%随机点图
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UR5
% %密集型工作空间散点图，通过扫描得到背部极端位姿逆运动学极端角度，取6角最大值构造工作空间
% Q=[q1;q2;q3;q4;q5];
% [Max_Q,Max_index]=max(Q,[],1);%the maximum value of q
% [Min_Q,Min_index]=min(Q,[],1);%the minimum value of q
% %通过限制每个关节最大最小角度规定UR5工作空间
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UR5
% theta1/theta2 overlap
% UR5 just reflect workspace of massage
% Build the area needed for massage through pictures
%%%%%%%%%%%%Use "for" cycle make up workspace%%%%%%%%%%%%%
% for th2=Min_Q(2):0.1:Max_Q(2)
%     for th3=Min_Q(3):0.4:Max_Q(3)
%         for th4=Min_Q(4):0.4:Max_Q(4)
%             for th5=Min_Q(5):0.4:Max_Q(5)
%                  move_one_step(th1,th2,th3,th4,th5,th6,0);
%                  robotxyz=robot.fkine([th1,th2,th3,th4,th5,th6]);
%                  plot3(robotxyz.t(1),robotxyz.t(2),robotxyz.t(3),'r.');hold on;
%             end
%         end
%     end
% end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UR5andMassage_robot
%限制每个关节，取随机点
% theta1=unifrnd(Min_Q(1),Max_Q(1),[1,30000]);%第一关节变量限位
% theta2=unifrnd(Min_Q(2),Max_Q(2),[1,30000]);%第二关节变量限位
% theta3=unifrnd(Min_Q(3),Max_Q(3),[1,30000]);%第三关节变量限位
% theta4=unifrnd(Min_Q(4),Max_Q(4),[1,30000]);%第四关节变量限位
% theta5=unifrnd(Min_Q(5),Max_Q(5),[1,30000]);%第五关节变量限位
% theta6=unifrnd(Min_Q(6),Max_Q(6),[1,30000]);%第六关节变量限位
% G = cell(30000, 6);%建立元胞数组
% for n = 1:30000
%     G{n} =[theta1(n) theta2(n) theta3(n) theta4(n) theta5(n) theta6(n)];
% end%产生30000组随机点
% H1=cell2mat(G);%将元胞数组转化为矩阵
% T=double(robot.fkine(H1));%机械臂正解
% scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)))%随机点图
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Test
% %限制每个关节，取随机点
% theta1=unifrnd(-pi,pi,[1,30000]);%第一关节变量限位
% theta2=unifrnd(-pi,pi,[1,30000]);%第二关节变量限位
% theta3=unifrnd(-pi,pi,[1,30000]);%第三关节变量限位
% theta4=unifrnd(5,5,[1,30000]);%第四关节变量限位
% theta5=unifrnd(-pi,pi,[1,30000]);%第五关节变量限位
% theta6=unifrnd(-pi,pi,[1,30000]);%第六关节变量限位
% G = cell(30000, 6);%建立元胞数组
% for n = 1:30000
%     G{n} =[theta1(n) theta2(n) theta3(n) theta4(n) theta5(n) theta6(n)];
% end%产生30000组随机点
% H1=cell2mat(G);%将元胞数组转化为矩阵
% T=double(robot.fkine(H1));%机械臂正解
% scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)))%随机点图





