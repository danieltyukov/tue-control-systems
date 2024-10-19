%%%%%% The MSDM system %%%%%%%%%%%%%%%
clear all;
J1 = 5.8198*10.^-6;  % Moment of Inertia of the first mass(kg m^2)
J2 = 4.769*10.^-6; % Moment of Inertia of the second mass(kg m^2)
k = 0.2656; % Torsional damping of the shaft () 
d = 3.125*10.^-5; % Torsional damping of the shaft (Nm/rad)
Km = 4.4*10.^-2; % Motor Constant (Nm/A)
b = 1*10.^-5; % Viscous friction (Nms/rad)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Fill in the transfer functions
s = tf('s');
G = (Km*(J2*s^2+(b+d)*s+k))/(J1*J2*s^4+(J1+J2)*(d+b)*s^3+((J1+J2)*k+b^2+2*b*d)*s^2+2*b*k*s); % fill in the transfer functions
H = 0.1; % fill in the transfer functions
D = 0.1; % fill in the transfer functions

%% 2. Compute open and closed loop transfer functions
L = D*G*H; % fill in the transfer functions
T = (D*G*H)/(1+D*G*H); % fill in the transfer functions
Gs = (G*H)/(1+D*G*H); % fill in the transfer functions

%% 3. Steady State Error
ss_error =  0;