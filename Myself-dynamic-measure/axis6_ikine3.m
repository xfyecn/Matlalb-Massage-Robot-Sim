function p = axis6_ikine3(temp,zitai,sd)

%=============================================
%Inverse Kinematics
%=============================================
L0 = 0.178; L1 = 0.205; L2 = 0.105; L3 = 0.105; L4 = 0.1315;
ax = temp(1,3); ay = temp(2,3); az = temp(3,3);
px = temp(1,4); py = temp(2,4); pz = temp(3,4);
nx = temp(1,1); ny = temp(2,1); nz = temp(3,1);
ox = temp(1,2); oy = temp(2,2); oz = temp(3,2);
%Itheta0
%Itheta0取值范围-360->360
a = L4*ax-px;
b = py-L4*ay;
if(b ~= 0)
	Itheta0temp = atan(a/b)*180/3.1415926;
	% if(a >= 0 && b > 0)
		% Itheta0(1) = Itheta0temp-180;
		% Itheta0(2) = Itheta0temp;
	% elseif(a >= 0 && b < 0)
		% Itheta0(1) = Itheta0temp+180;
		% Itheta0(2) = Itheta0temp;	
	% elseif(a <= 0 && b > 0)
		% Itheta0(1) = Itheta0temp+180;
		% Itheta0(2) = Itheta0temp;
	% elseif(a <= 0 && b < 0)
		% Itheta0(1) = Itheta0temp-180;
		% Itheta0(2) = Itheta0temp;	
	% end
	if(a <= 0 && b < 0)
		Itheta0(1) = Itheta0temp;
	elseif(a <= 0 && b > 0)	
		Itheta0(1) = Itheta0temp+180;
	elseif(a >= 0 && b < 0)
		Itheta0(1) = Itheta0temp;
	elseif(a >= 0 && b > 0)	
		Itheta0(1) = Itheta0temp-180;
	end
	if(Itheta0(1) < 0) Itheta0(2) = Itheta0(1)+180;
	else Itheta0(2) = Itheta0(1)-180;
	end
else
	if(a ~= 0)
		Itheta0(1) = 90;
		Itheta0(2) = -90;	
	else
		Itheta0(1) = 0;
		Itheta0(2) = 180;		
	end
end
if(Itheta0(1) < 0)
	if(abs(Itheta0(1)+360-sd(1)) < abs(Itheta0(1)-sd(1)))
		Itheta0(1) = Itheta0(1)+360;
	end
else
	if(abs(Itheta0(1)-360-sd(1)) < abs(Itheta0(1)-sd(1)))
		Itheta0(1) = Itheta0(1)-360;
	end
end
if(Itheta0(2) < 0)
	if(abs(Itheta0(2)+360-sd(1)) < abs(Itheta0(2)-sd(1)))
		Itheta0(2) = Itheta0(2)+360;
	end
else
	if(abs(Itheta0(2)-360-sd(1)) < abs(Itheta0(2)-sd(1)))
		Itheta0(2) = Itheta0(2)-360;
	end
end
	
% disp(['Itheta0(1) = ',num2str(Itheta0(1)),' Itheta0(2) = ',num2str(Itheta0(2))]);


%==============================================
%Theta1 2取值范围-180->180未考虑机械
%Itheta1 Itheta2
for i=1:1:2
	a = pz-az*L4-L0;
	b = -(cosd(Itheta0(i))*py-L4*(cosd(Itheta0(i))*ay-sind(Itheta0(i))*ax)-sind(Itheta0(i))*px);
	Itheta2(2*i-1) = acos((a^2+b^2-L1^2-(L2+L3)^2)/(2*L1*(L2+L3)))*180/pi;
	Itheta2(2*i) = -Itheta2(2*i-1) ;
	Itheta1temp1 = acos((a^2+b^2+L1^2-(L2+L3)^2)/(2*L1*sqrt(a^2+b^2)))*180/pi;
	if(a ~= 0)
		if(a < 0 && b >= 0)
			Itheta1temp = atan(b/a)*180/pi+180;
		elseif(a < 0 && b < 0)
			Itheta1temp = -180+atan(b/a)*180/pi;
		else	
			Itheta1temp = atan(b/a)*180/pi;
		end
	else
		if(b > 0)
			Itheta1temp = 90;
		else
			Itheta1temp = -90;
		end		
	end
	Itheta1(2*i-1) = Itheta1temp-Itheta1temp1;  %取值与Itheta2的两个值必须对应
	Itheta1(2*i) = Itheta1temp+Itheta1temp1;
	if(Itheta1(2*i-1) > 180) Itheta1(2*i-1) = Itheta1(2*i-1)-360;
	elseif(Itheta1(2*i-1) < -180) Itheta1(2*i-1) = Itheta1(2*i-1)+360;
	end
	if(Itheta1(2*i) > 180) Itheta1(2*i) = Itheta1(2*i)-360;
	elseif(Itheta1(2*i) < -180) Itheta1(2*i) = Itheta1(2*i)+360;
	end
end	

%==============================================
%Itheta4
d = az;
c = cosd(Itheta0(1))*ay - sind(Itheta0(1))*ax;
S12 = sind(Itheta1(1)+Itheta2(1));
C12 = cosd(Itheta1(1)+Itheta2(1));
Itheta4(1) = acos(C12*d-S12*c)*180/3.1415926;
Itheta4(2) = -Itheta4(1);
c = cosd(Itheta0(1))*ay - sind(Itheta0(1))*ax;
S12 = sind(Itheta1(2)+Itheta2(2));
C12 = cosd(Itheta1(2)+Itheta2(2));
Itheta4(3) = acos(C12*d-S12*c)*180/3.1415926;
Itheta4(4) = -Itheta4(3);
c = cosd(Itheta0(2))*ay - sind(Itheta0(2))*ax;
S12 = sind(Itheta1(3)+Itheta2(3));
C12 = cosd(Itheta1(3)+Itheta2(3));
Itheta4(5) = acos(C12*d-S12*c)*180/3.1415926;
Itheta4(6) = -Itheta4(5);
c = cosd(Itheta0(2))*ay - sind(Itheta0(2))*ax;
S12 = sind(Itheta1(4)+Itheta2(4));
C12 = cosd(Itheta1(4)+Itheta2(4));
Itheta4(7) = acos(C12*d-S12*c)*180/3.1415926;
Itheta4(8) = -Itheta4(7);		
		
%==============================================
%Itheta3
%Itheta3取值范围-360->360
for i=1:1:4
	if(i>=3)
		S0 = sind(Itheta0(2)); C0 = cosd(Itheta0(2));
	else
		S0 = sind(Itheta0(1)); C0 = cosd(Itheta0(1));
	end
	S1 = sind(Itheta1(i)); C1 = cosd(Itheta1(i));
	S2 = sind(Itheta2(i)); C2 = cosd(Itheta2(i));
	S12 = sind(Itheta1(i)+Itheta2(i));
	C12 = cosd(Itheta1(i)+Itheta2(i));
	a = (C0*px) + (S0*py);
	b = L1*S2 + (S0*px-C0*py)*C12 + (L0-pz)*S12;
	if( b ~= 0)
		te = atan(a/b)*180/pi;
		if(a <= 0 && b < 0)
			Itheta3(2*i-1) = te;
		elseif(a <= 0 && b > 0)	
			Itheta3(2*i-1) = te+180;
		elseif(a >= 0 && b < 0)
			Itheta3(2*i-1) = te;
		elseif(a >= 0 && b > 0)	
			Itheta3(2*i-1) = te-180;
		end
		if(Itheta3(2*i-1) < 0) Itheta3(2*i) = Itheta3(2*i-1)+180;
		else Itheta3(2*i) = Itheta3(2*i-1)-180;
		end
	else
		if(a ~= 0)
			Itheta3(2*i) = 90;
			Itheta3(2*i-1) = -90;
		else
			Itheta3(2*i) = 0;
			Itheta3(2*i-1) = 180;
		end
	end
end
for i=1:1:8
	if(Itheta3(i) < 0)
		if(abs(Itheta3(i)+360-sd(4)) < abs(Itheta3(i)-sd(4)))
			Itheta3(i) = Itheta3(i)+360;
		end
	else
		if(abs(Itheta3(i)-360-sd(4)) < abs(Itheta3(i)-sd(4)))
			Itheta3(i) = Itheta3(i)-360;
		end
	end
end

%==============================================
%Itheta5
%Itheta5取值范围-360->360
for i=1:1:4
	if(i>=3)
		S0 = sind(Itheta0(2)); C0 = cosd(Itheta0(2));
	else
		S0 = sind(Itheta0(1)); C0 = cosd(Itheta0(1));
	end
	S1 = sind(Itheta1(i)); C1 = cosd(Itheta1(i));
	S2 = sind(Itheta2(i)); C2 = cosd(Itheta2(i));
	S12 = sind(Itheta1(i)+Itheta2(i));
	C12 = cosd(Itheta1(i)+Itheta2(i));
	a = oz*C12 + (S0*ox - C0*oy)*S12;
	b = -nz*C12 + (C0*ny - S0*nx)*S12;
	if( b ~= 0)
		te = atan(a/b)*180/pi;
		if(a <= 0 && b < 0)
			Itheta5(2*i) = te;
		elseif(a <= 0 && b > 0)	
			Itheta5(2*i) = te+180;
		elseif(a >= 0 && b < 0)
			Itheta5(2*i) = te;
		elseif(a >= 0 && b > 0)	
			Itheta5(2*i) = te-180;
		end
		if(Itheta5(2*i) < 0) Itheta5(2*i-1) = Itheta5(2*i)+180;
		else Itheta5(2*i-1) = Itheta5(2*i)-180;
		end
	else
		if(a ~= 0)
			Itheta5(2*i-1) = 90;
			Itheta5(2*i) = -90;
		else
			Itheta5(2*i-1) = 0;
			Itheta5(2*i) = 180;
		end
	end
end
for i=1:1:8
	if(Itheta5(i) < 0)
		if(abs(Itheta5(i)+360-sd(6)) < abs(Itheta5(i)-sd(6)))
			Itheta5(i) = Itheta5(i)+360;
		end
	else
		if(abs(Itheta5(i)-360-sd(6)) < abs(Itheta5(i)-sd(6)))
			Itheta5(i) = Itheta5(i)-360;
		end
	end
end

		
%===============================================
%theta0
ptemp(1,1) = Itheta0(1);
ptemp(2,1) = Itheta0(1);
ptemp(3,1) = Itheta0(1);
ptemp(4,1) = Itheta0(1);
ptemp(5,1) = Itheta0(2);
ptemp(6,1) = Itheta0(2);
ptemp(7,1) = Itheta0(2);
ptemp(8,1) = Itheta0(2);
%theta1
ptemp(1,2) = Itheta1(1);
ptemp(2,2) = Itheta1(1);
ptemp(3,2) = Itheta1(2);
ptemp(4,2) = Itheta1(2);
ptemp(5,2) = Itheta1(3);
ptemp(6,2) = Itheta1(3);
ptemp(7,2) = Itheta1(4);
ptemp(8,2) = Itheta1(4);
%theta2
ptemp(1,3) = Itheta2(1);
ptemp(2,3) = Itheta2(1);
ptemp(3,3) = Itheta2(2);
ptemp(4,3) = Itheta2(2);
ptemp(5,3) = Itheta2(3);
ptemp(6,3) = Itheta2(3);
ptemp(7,3) = Itheta2(4);
ptemp(8,3) = Itheta2(4);
%theta3
ptemp(1:8,4) = Itheta3(1:8);
%theta4
ptemp(1:8,5) = Itheta4(1:8);
%theta5
ptemp(1:8,6) = Itheta5(1:8);
%===========================================================	
%姿态分组
%左臂 theta0 > 0    ==== L/R
%下臂 theta2 > 0	  ==== A/B
%翻转 theta3 > 0    ==== F/N
%1 -> LAF
%2 -> LAN
%3 -> LBF
%4 -> LBN
%5 -> RAF
%6 -> RAN
%7 -> RBF
%8 -> RBN	  
ptemp
% for i = 1:1:8
	% dd = 0;
	% if(ptemp(i,2) < 0) dd = dd+4; end
	% if(ptemp(i,3) < 0) dd = dd+2; end %上下臂
	% if(ptemp(i,5) < 0) dd = dd+1; end
	% dtemp(dd+1,1:6) = ptemp(i,1:6);
% end	
p = ptemp;
%p(1:6) = ptemp(zitai,1:6);
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  