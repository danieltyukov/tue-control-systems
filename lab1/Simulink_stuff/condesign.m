%clear workspace
clear

%% plant model parameters
%%%%%% MSDM parameters %%%%%%%%%%%%%%%%%%%%%%%%%
J1 = 5.8198*10.^-6;  % Moment of Inertia of the first mass(kg m^2)
J2 = 4.769*10.^-6; % Moment of Inertia of the second mass(kg m^2)

k = 0.2656; % Torsional damping of the shaft () 
d = 3.125*10.^-5; % Torsional damping of the shaft (Nm/rad)
Km = 4.4*10.^-2; % Motor Constant (Nm/A)
b = 1*10.^-5; % Viscous friction (Nms/rad)


  
%% model of the plant 
%Im to theta1
s = tf('s');
G = (Km*(J2*s^2+(b+d)*s+k))/(J1*J2*s^4+(J1+J2)*(d+b)*s^3+((J1+J2)*k+b^2+2*b*d)*s^2+2*b*k*s); % fill in the transfer functions

%theta1 to theta2
G2=tf([d k],[J2 (b+d) k]);


%% Design controller
% Frequency of Low pass measurement filter H(s)
H = tf([200*pi],[1, 200*pi]);

% Proportional gain controller
omega_num=30; %30
zeta=0.5;
omega_den=800; %800
zeta_den=0.5;

%D=0.002361*(1+0.54*s)*(1+7.382*s)/((1+8.804*s)*(1+0.000123*s));

%D=0.0002833*(1+0.58*s)*(1+12*s)/(s*(1+0.00011*s)*(1+0.016*s)); %BEST ONE

%D=0.0002833*(1+0.69*s)*(1+27*s)/(s*(1+0.0023*s)*(1+0.011*s)); %BEST BEST ONE

%D=0.0002833*(1+0.57*s)*(1+35*s)/(s*(1+0.0023*s)*(1+0.0088*s)); % BEST 3 ONE

D=0.0002833*(1+0.63*s)*(1+35*s)/(s*(1+0.0033*s)*(1+0.0088*s)); %Upload


%controlSystemDesigner(G*H,D)

% put all designed transfer functions in a .mat file
save controller.mat

%% Load connections
load('RL_FA_busses_5ESD0.mat')
%% load tunable parameters
load('RL_FA_controller_5ESD0.mat')
