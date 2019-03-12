
%建立UR5 D-H模型
%       theta    d        a        alpha     offset
global L1 L2 L3 L4 L5 L6 
global robot
L1=Link([0       .892   0        0         0     ]); %定义连杆的D-H参数
L2=Link([-pi/2   0      0        pi/2      0     ]);
L3=Link([0       0      -4.25     0         0     ]);
L4=Link([-pi/2   1.093  -3.92     0         0     ]);
L5=Link([0       .9475  0        pi/2      0     ]);
L6=Link([0       4.25    0        0         0     ]);
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','UR5_Massage');
robot.comment='LY';
robot.display();
% teach(robot);
% theta=[0 0 0 0 0 0];
% robot.plot(theta);