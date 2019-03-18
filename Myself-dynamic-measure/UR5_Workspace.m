close all;
% A(1:90)=struct('cdata',[],'colormap',[]);
clear;

global Link
UR5_Mdl;

th=[0,0,0,0,0,0];
grid on;

for th1=1:1:4
    for th2= -110:4:110
        for th3=-90:4:70
 %             for th4=-160:50:160
 %                for th5=-120:60:120
 %                    for th6=-226:50:226                       
                            Link(2).th=0*pi+th1*pi/180; 
                            Link(3).th=-0.5*pi+th2*pi/180;
                            Link(4).th=0*pi+th3*pi/180; fprintf('%d %d %d  \n',[th1,th2,th3]');
                            Link(5).th=0*pi/180;
                            Link(6).th=0*pi/180;
                            Link(7).th=0*pi/180;  fprintf('%d %d %d %d %d %d  \n',[th1,th2,th3,0,0,0]');  
%                         Link(4).th=th4*pi/180;
%                         Link(5).th=th5*pi/180; fprintf('%d %d %d %d %d  \n',[th1,th2,th3,th4,th5]');
%                         Link(6).th=th6*pi/180; fprintf('%d %d %d %d %d %d
%                         \n',[th1,th2,th3,th4,th5,th6]');
                        for i=1:7
                            Matrix_DH(i);
                        end
                        for i=2:7
                            Link(i).A=Link(i-1).A*Link(i).A;
                            Link(i).p= Link(i).A(:,4); 
                        end
                        grid on;  
                        plot3(Link(6).p(1),Link(6).p(2),Link(6).p(3),'r*');pause(0.0001);hold on;
%                     end
 %                 end
 %             end
        end
    end
end

  
                        
 



% for d1=0:10:200
%     for th=-120:10:120
%         for d2=0:10:100
%             thx=th*pi/180;
%             A1=[ 1 0 0 0;
%                  0 1 0 0; 
%                  0 0 1 d1;
%                  0 0 0 1];
%             A2=[cos(thx) -sin(thx) 0 0;
%                 sin(thx) cos(thx) 0 0; 
%                 0 0 1 0; 
%                 0 0 0 1];
%             A3=[ 1 0 0 d2;
%                  0 1 0 0; 
%                  0 0 1 0;
%                  0 0 0 1];
%             
%             p=A1*A2*A3;
%             plot3(p(1,4),p(2,4),p(3,4)); hold on;
%         end
%     end
% end

    





