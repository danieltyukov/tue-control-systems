%%%%%% The MSDM system parameters %%%%%%%%%%%%%%%
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
H = (200*pi)/(s+200*pi); % fill in the transfer functions
D = s; % fill in the transfer functions

%% [optional]: Compute open and closed loop transfer functions
L = D*G*H; % fill in the transfer functions
T = (D*G*H)/(1+D*G*H); % fill in the transfer functions
Gs = (G*H)/(1+D*G*H); % fill in the transfer functions

% Test T1 (Fill in the correct transfer T and Gs to get the right results of test T1 and T2)
sys = [T, Gs];
t = 0:0.01:20; % check end time
T1 = [200*ones(size(t)); 0.2*sin(2*pi*t)];
y = lsim(sys,T1,t);
plot(t,y)

% Test T2
T2 = [200*ones(size(t));0.1*ones(size(t))];
y = lsim(sys,T2,t);
plot(t,y)