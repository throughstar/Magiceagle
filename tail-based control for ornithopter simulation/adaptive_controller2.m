function [sys,x0,str,ts,simStateCompliance] = adaptive_controller2(t,x,u,flag,para)
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
sizes.NumOutputs     = 5;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
 
%
% initialize the initial conditions
%
x0  = para.theta0;
 
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
%theta = x(1:4);
%gamma_ref = u(1);
gamma_ref = u(1);
r = u(2);
alpha = u(3);
%alpha = z2 - z1 + alpha_0;
%gamma = z1 + gamma_ref;
%q = z3;
q = u(4);
Gamma = [-0.003 0 0 0;0 -0.005 0 0;0 0 -0.005 0;0 0 0 -0.0005];
Lambda = 2;
%phi = u(8:11);

%system discription
phi = [1;alpha;q;(Lambda*(q + r - gamma_ref)/para.beta_r)];
dot_theta = -para.beta_r * (q + r - gamma_ref) * Gamma * phi;
sys(1:4) = dot_theta;

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
theta = x(1:4);
gamma_ref = u(1);
r = u(2);
alpha = u(3);
q = u(4);
Lambda = 3;
phi = [1;alpha;q;(Lambda*(q + r - gamma_ref)/para.beta_r)];
delta_e = -transpose(theta) * phi;
sys(1:4) = theta;
sys(5) = delta_e;

 
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