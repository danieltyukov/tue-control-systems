%clc;
%clear;
%close all;

%% ************** 5ESD0 - Control Systems, Lab 2 - Student file **************
load('RL_ML_busses.mat'); % Load in busses if simulation is also run from this file
s = tf('s');

%% ************** SYSTEM PARAMETERS ****************************************
Ts   = 1/5000;  % fast sampletime for the model
Tslf = 1*Ts;    % sample time of the ZOH 
R    = 5;       % (ohm)
cL   = 1.9e-3;  % (m)
Ld   = 55e-3;   % (H)
m    = 0.018;   % (kg)
g    = 9.81;    % (m/s^2)
Linf = 0.1;     % (H)


% In the part below, uncomment and fill in the missing code at the places indicated with ... 
% Note that you are allowed to add more code if necesarry
%% ************** LINEARIZED PLANT MODEL *********************************** 
%
%    f(x,u) ~ f(x0,u0) + d/dx f(x0,u0)(x-x0) + d/du f(x0,u0)(u-u0)
% 
% ************************************************************************* 

% Non-Linear plant model
f = @(x1,x2,x3,u) [((u-R*x1+x3*x1*Ld*cL/((cL+x2)^2))/(Linf+Ld*cL/(cL+x2))),(x3),(g-(Ld*cL*(x1^2))/(2*m*((cL+x2)^2)))].';

% operating point
Y0 = 10e-3;   % DO NOT change this value! 

Ydot0 = 0;   % Speed of the ball in operating point
I0    = sqrt((2*g*m)/(Ld*cL))*(cL+Y0);   % Current in operating point
U0    = R*I0;   % Voltage in operating point

x0    = [I0,Y0,Ydot0].';   % State of the system in the operating point

% Inductance in operating point
L0    = Linf+(Ld*cL)/(cL+Y0);   % Inductance in operating point
dL0dy = -(Ld*cL)/((cL+Y0)^2);   % Derivative of the Inductance in operating point wrt the position

% check f(x0,u0) = 0;
% f0 = f(I0,Y0,Ydot0,U0);

% Create ss matrices A, B, C and D
A = [-R/L0, 0, (Ld*cL*I0)/(L0*((cL+Y0)^2)); 
     0, 0, 1; 
    (-Ld*cL*I0/(m*((cL+Y0)^2))), (Ld*cL*(I0^2)/(m*((cL+Y0)^3))), 0];B = [1/L0,0,0].';
C = [0,1,0];
D = 0;

% Plant transfer function:
[num, den] = ss2tf(A, B, C, D);
G = tf(num, den);

%% ******************** Create state feedback controller and observer *********************************
% Check controlability/observability (rank used to check whether obseravable or controllable)
controllability = rank(ctrb(A, B));
observability = rank(obsv(A, C));

% check whether controllable and observable
if controllability == size(A, 1) && observability == size(A, 1)
    disp('System is controllable and observable')
else
    disp('System is not controllable and/or observable')
end

%% Statefeedback
tn = [0 0 1]*inv([B A*B (A^2)*B]);
T  = inv([tn*A^2; tn*A; tn]); % this is basically T-1 inverted...

% Transformation into Controllable Canonical Form
[Ac, Bc, Cc, Dc] = tf2ss(num, den);  % Convert TF to state-space in canonical form

% Pole placement for state feedback controller
poles = [-1000, -60, -30];   % Desired pole locations
K = place(A, B, poles);      % State feedback gain matrix

% Print the state feedback gain K
disp('State Feedback Gain K:');
disp(K);

% Verify that the closed-loop poles are at the desired locations
Acl = A - B * K;                     % Closed-loop system matrix
closed_loop_poles = eig(Acl);        % Closed-loop eigenvalues
disp('Closed-loop Eigenvalues with State Feedback:');
disp(closed_loop_poles);

%% State observer
%% State Observer
% Desired observer poles
observer_poles = [-1000, -200, -100];  % Desired observer poles

% Compute observer gain L
L = place(A', C', observer_poles)';

% Print the observer gain L
disp('Observer Gain L:');
disp(L);

% Verify that the observer poles are at the desired locations
A_observer = A - L * C;              % Observer error dynamics matrix
observer_eigenvalues = eig(A_observer);
disp('Observer Error Dynamics Eigenvalues:');
disp(observer_eigenvalues);


%% Integral action
% Augmented state space (with integral state x_I)
A_ = [0, C; zeros(size(A, 1), 1), A];
B_ = [D; B];
C_ = [0, C];
D_ = D;

% Find K_ = [Ki K]
P = [-1000 -800 -60 -30];
K_ = place(A_, B_, P);
Ki  = K_(1);
K   = K_(2:4);

% Print the controller gains
disp('Integral Gain Ki:');
disp(Ki);
disp('State Feedback Gain K with Integral Action:');
disp(K);

% Verify that the closed-loop poles are at the desired locations
Acl_aug = A_ - B_ * K_;
closed_loop_poles_aug = eig(Acl_aug);
disp('Closed-loop Eigenvalues with Integral Action:');
disp(closed_loop_poles_aug);

%% specifications
tr = 0.05;  % [s]
Mp = 0.16;  % [%]
ts = 0.3;   % [s]

%{
p1  = -165;
p2  = -100;
P   = [-1000 -800 p1 p2];
K_  = K_ = place(A_, B_, P);
Ki  = K_(1);
K   = K_(2:4);
%}

%% Reference Tracking (not included in lab!)
N  = 0;
M  = 0;

%% Bus variables 
disable_PWM=0;
I_Offset_cor=0;
terminate=0;

%% save all variables
save controller.mat 

%% auto run sim
sim('RL_ML_TOP_SIM.slx')