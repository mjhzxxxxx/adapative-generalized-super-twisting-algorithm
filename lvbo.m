 function [sys,x0,str,ts] =observor(t,x,u,flag)

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
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 1;   
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes);
x0=[0];
str = [];
ts  = [0 0];


function sys=mdlDerivatives(t,x,u)
global tau m n
ua=u(1);
ueq=x(1);
dueq=(1/tau)*((abs((ua-ueq))^m)*sign(ua-ueq)+(abs((ua-ueq))^n)*sign(ua-ueq));
%dueq=(1/tau)*((abs((ua-ueq)))*sign(ua-ueq));
sys(1)=dueq;     

function sys=mdlOutputs(t,x,u)
global aerfa tau n m
ua=u(1);
ueq=x(1);
dueq=(1/tau)*((abs((ua-ueq))^m)*sign(ua-ueq)+(abs((ua-ueq))^n)*sign(ua-ueq));
%dueq=(1/tau)*((abs((ua-ueq)))*sign(ua-ueq));
sys(1)=x(1);
sys(2)=dueq/aerfa;







