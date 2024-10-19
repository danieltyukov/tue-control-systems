clc;
clear;
close all;

%% ************** 5ESD0 - Control Systems, Lab 2 - Student file **************
% load('RL_ML_busses.mat'); % Load in busses if simulation is also run from this file
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
A = [-R/L0,0,(Ld*cL*I0)/(L0*((cL+Y0)^2));0,0,1;(-Ld*cL*I0/(m*((cL+Y0)^2))),(Ld*cL*(I0^2)/(m*((cL+Y0)^3))),0];
B = [1/L0,0,0].';
C = [0,1,0];
D = 0;

% Plant transfer function:
s = tf('s');    % Laplace variable s
G = C * ((s * eye(size(A)) - A) \ B) + D;

%% ******************** Create state feedback controller and observer *********************************
% Check controlability/observability 
controllability = ctrb(A, B);
observability = obsv(A, C);

%% Statefeedback
% Transformation matrix T
tn = [0 0 1]*inv([B A*B (A^2)*B]);
T  = inv([tn*A^2; tn*A; tn]);

% Controllable canonical form
Ac = inv(T)*A*T;
Bc = inv(T)*B;
Cc = C*T;
Dc = D;

% pole locations (defined as (s+p1)(s+p2)(s+p3))
p1 = 1000;
p2 = 60;
p3 = 30;

syms k1 k2 k3 s

% Extract coefficients alpha_1, alpha_2, alpha_3
expand((s+p1)*(s+p2)*(s+p3));
alpha_1 = p1 + p2 + p3;
alpha_2 = p1*p2 + p1*p3 + p2*p3;
alpha_3 = p1*p2*p3;

syms K [1 3];
char_poly =expand(det(s*eye(3)-A+B*K));
[coeffs_char_poly, terms_char_poly] = coeffs(char_poly, s);

% Desired characteristic polynomial coefficients

gamma_1 = coeffs_char_poly(end-2);  % Coefficient of s^2
gamma_2 = coeffs_char_poly(end-1);  % Coefficient of s^1
gamma_3 = coeffs_char_poly(end);  % Constant term

% Set up equations to solve for k1, k2, k3
eq1 = alpha_1 == gamma_1;
eq2 = alpha_2 == gamma_2;
eq3 = alpha_3 == gamma_3;

S = solve([eq1, eq2, eq3]);
k1 = eval(S.K1);
k2 = eval(S.K2);
k3 = eval(S.K3);

% State feedback gain K
K = [k1, k2, k3]

%% State observer
% Apply the same method as for statefeedback but now with the observability
% canonical form 

% No need to transform to canonical form; proceed directly

% Pole locations (defined as (s + p1)(s + p2)(s + p3))
p1 = 1000;
p2 = 200;
p3 = 100;

syms l1 l2 l3 s

% Desired characteristic polynomial coefficients
desired_char_poly = expand((s + p1)*(s + p2)*(s + p3));

% Extract coefficients gamma_1, gamma_2, gamma_3
[coeffs_desired_char_poly, ~] = coeffs(desired_char_poly, s);
gamma_1 = coeffs_desired_char_poly(end - 2);  % Coefficient of s^2
gamma_2 = coeffs_desired_char_poly(end - 1);  % Coefficient of s^1
gamma_3 = coeffs_desired_char_poly(end);      % Constant term

% Compute A - L*C
L_sym = [l1; l2; l3];

A_obs = A - L_sym*C;

% Compute characteristic polynomial of A_obs
char_poly = det(s*eye(3) - A_obs);
char_poly = expand(char_poly);

% Extract coefficients alpha_1, alpha_2, alpha_3
[coeffs_char_poly, ~] = coeffs(char_poly, s);
alpha_1 = coeffs_char_poly(end - 2);
alpha_2 = coeffs_char_poly(end - 1);
alpha_3 = coeffs_char_poly(end);

% Set up equations to solve for l1, l2, l3
eq1 = alpha_1 == gamma_1;
eq2 = alpha_2 == gamma_2;
eq3 = alpha_3 == gamma_3;

% Solve the equations
S = solve([eq1, eq2, eq3], [l1, l2, l3]);

% Evaluate the solutions
l1 = double(S.l1);
l2 = double(S.l2);
l3 = double(S.l3);

% Observer gain L
L = [l1; l2; l3]

%% Integral action
% Augmented state space (with integral state x_I)
% A_ = ...;
% B_ = ...;
% C_ = ...;
% D_ = ...;

% Find K_ = [Ki K]
% P = [-1000 -800 -60 -30];
% K_  = ...;
% Ki  = K_(1);
% K   = K_(2:4);

%% specifications
tr = 0.05;  % [s]
Mp = 0.16;  % [%]
ts = 0.3;   % [s]

% p1  = ...;
% p2  = ...;
% P   = [-1000 -800 p1 p2];
% K_  = ...;
% Ki  = K_(1);
% K   = K_(2:4);

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
% sim('RL_ML_TOP_SIM.slx')