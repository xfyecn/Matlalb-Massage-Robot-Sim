%建立机器人模型
global L1 L2 L3 L4 L5 L6
global robot
L1=Link('d',0,'a',5.5,'alpha',0);
L2=Link('d',0,'a',4.5,'alpha',0);
L3=Link('d',0,'a',0,'alpha',-pi);
L4=Link('theta',0,'a',0,'alpha',0);%三自由度机械臂，最后末端加入工具或者移动关节，整体四自由度
L5=Link('d',0,'a',0,'alpha',0);
L6=Link('d',0,'a',0,'alpha',0);
L4.qlim=[0 5];%移动关节长度限制
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Massage_Robot');
% q=[pi/2 0 0 0 0 0];
% robot.plot(q);
