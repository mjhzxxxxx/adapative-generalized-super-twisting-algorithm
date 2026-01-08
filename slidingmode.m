 function [sys,x0,str,ts] =space_robot(t,x,u,flag)

switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case {2,4,9}
    sys=[];   
  case 3
    sys=mdlOutputs(t,x,u);
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;   
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
x0=[5 0 0 3];
str = [];
ts  = [0 0];


function sys=mdlDerivatives(t,x,u)

global  gama1 gama2 r p aerfa K1 K2 m n aibuxilong miu v nete 
ueq=abs(u(1));
xigama=x(1);
xigamajifen=x(2);
gama3=x(3);
fai=x(4);
%d=2*sin(0.2*pi*t)+0.15*sin(2*pi*t);
d=sin(t);
ua=-gama1*(abs(xigama))^p*sign(xigama)-gama2*(abs(xigama))^r*sign(xigama)-xigamajifen;
dxigama=ua+d;
dxigamajifen=(gama3+nete)*sign(xigama);
X1=gama3-ueq/aerfa-aibuxilong;
dgama3=-K1*(abs(X1))^miu*sign(X1)-K2*(abs(X1))^v*sign(X1)-fai*sign(X1);
dfai=0;
sys(1)=dxigama;
sys(2)=dxigamajifen;
sys(3)=dgama3;
sys(4)=dfai;

function sys=mdlOutputs(t,x,u)
global aerfa aibuxilong nete gama1 gama2 r p
ueq=abs(u(1));
xigama=x(1);
xigamajifen=x(2);
ua=-gama1*(abs(xigama))^p*sign(xigama)-gama2*(abs(xigama))^r*sign(xigama)-xigamajifen;
X1=x(3)-ueq/aerfa-aibuxilong;
sys(1)=x(1);
sys(2)=x(2);
sys(3)=(x(3)+nete)*sign(x(1));
%sys(4)=abs(2*0.2*pi*cos(0.2*pi*t)+0.15*2*pi*cos(2*pi*t));
sys(4)=abs(cos(t));
sys(5)=x(3);
sys(6)=X1;
sys(7)=x(4);
sys(8)=ueq;
sys(9)=ua;






