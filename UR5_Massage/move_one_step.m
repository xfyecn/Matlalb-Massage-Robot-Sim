function move_one_step(th1,th2,th3,th4,th5,th6,fcla)

global L1 L2 L3 L4 L5 L6 
global robot
% UR5mdl;
L1.theta=th1;
L2.theta=th2;
L3.theta=th3;
L4.theta=th4;
L5.theta=th5;
L6.theta=th6;
robot.plot([L1.theta L2.theta L3.theta L4.theta L5.theta L6.theta]);
% drawnow;
if(fcla)
    cla;
end