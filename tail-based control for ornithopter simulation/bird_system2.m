function [sys,x0,str,ts,simStateCompliance] = bird_system2(t,x,u,flag,para)
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
switch flag,
 
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(para);
 
  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,para);
 
  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);
 
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,para);
 
  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
 
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);
 
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
 
end
 
% end sfuntmpl
 
%
%=============================================================================
% mdlInitializeSizes
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(para)
sizes = simsizes;
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
 
%
% initialize the initial conditions
%
x0  = para.z0;
 
%
% str is always an empty matrix
%
str = [];
 
%
% initialize the array of sample times
%
ts  = [0 0];
simStateCompliance = 'UnknownSimState';
 
% end mdlInitializeSizes
 
%=============================================================================
% mdlDerivatives
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,para)
%state definition
z1 = x(1);
z2 = x(2);
z3 = x(3);
zv = x(4);
gamma_ref = u(1);
%alpha_0 = u(2);
%alpha = z2 - z1 + alpha_0;
%gamma = z1 + gamma_ref;
%q = z3;
theta = u(2:5);
delta = u(6);
FT = u(7);
theta_V = u(8:10);
Vr = u(11);
Vr_dot = u(12);
%phi = u(8:11);

C_mdel = 1/theta(4,1);
C_m0 = theta(1,1) * C_mdel;
C_malp = theta(2,1) * C_mdel;
C_mq = theta(3,1) * C_mdel;
alpha_0 = para.m * para.g/para.Cf * cos(gamma_ref);
alpha_1 = z2 - z1 + alpha_0;
phi_V = [1;alpha_1;alpha_1^2];
gamma_1 = z1 + gamma_ref;
%system discription
sys(1) = (para.Cf/(para.m * para.V)) * (z2-z1);
sys(2) = z3;
sys(3) = para.beta_r * (C_m0 + C_malp * (z2 - z1 + alpha_0) + C_mq * z3 + C_mdel * delta);
sys(4) = para.beta_1 * (zv + Vr)^2 * transpose(phi_V) * theta_V + FT * cos(alpha_1)/para.m - para.g * sin(gamma_1) - Vr_dot;


% end mdlDerivatives
 
%
%=============================================================================
% mdlUpdate
%=============================================================================
%
function sys=mdlUpdate(t,x,u)
 
sys = [];
 
% end mdlUpdate
 
%
%=============================================================================
% mdlOutputs
%=============================================================================
%
function sys=mdlOutputs(t,x,u,para)
z1 = x(1);
z2 = x(2);
z3 = x(3);
gamma_ref = u(1);
alpha_0 = para.m * para.g/para.Cf * cos(gamma_ref);
%alpha_0 = u(2);
alpha_1 = z2 - z1 + alpha_0;
q = z3;
r = z1 + gamma_ref;
theta_1 = alpha_1 + r;
sys(1) = r;
sys(2) = alpha_1;
sys(3) = q;
sys(4) = theta_1;

 
% end mdlOutputs
 
%
%=============================================================================
% mdlGetTimeOfNextVarHit
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
 
% end mdlGetTimeOfNextVarHit
 
%
%=============================================================================
% mdlTerminate
%=============================================================================
%
function sys=mdlTerminate(t,x,u)
 
sys = [];
 
% end mdlTerminate