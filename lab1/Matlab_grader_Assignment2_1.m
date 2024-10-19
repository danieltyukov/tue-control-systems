J1 = 5.8198*10.^-6;  % Moment of Inertia of the first mass(kg m^2)
J2 = 4.769*10.^-6; % Moment of Inertia of the second mass(kg m^2)
k = 0.2656; % Torsional damping of the shaft () 
d = 3.125*10.^-5; % Torsional damping of the shaft (Nm/rad)
Km = 4.4*10.^-2; % Motor Constant (Nm/A)
b = 1*10.^-5; % Viscous friction (Nms/rad)

s=tf('s');
G = (Km*(J2*s^2+(b+d)*s+k))/(J1*J2*s^4+(J1+J2)*(d+b)*s^3+((J1+J2)*k+b^2+2*b*d)*s^2+2*b*k*s); % fill in the transfer functions
H = (200*pi)/(s+200*pi);
sys=G*H;

omega_num=30; %30
zeta=0.5;
omega_den=800; %800
zeta_den=0.5;
%D = 0.00099765*((s/omega_num)^2+2*zeta*s/omega_num+1)/((s/omega_den)^2+2*zeta_den*s/omega_den+1);
%D=0.002338*(1+0.5*s)*(1+7.4*s)/((1+8.8*s)*(1+0.00013*s));

%D=0.0029*(1+0.5*s)*(1+7.4*s)/((1+8.8*s)*(1+0.051*s)); %- upload this one

%D=0.001816*(1+0.77*s)*(1+3.8*s)/(s*(1+0.00011*s)*(1+0.017*s)); %if the upper one doesnt work continue with this one (it has an integrator)

%D1=6040.2*(s+0.2717)*(s+1.455)*(s^2+0.07498*s+9.228*exp(04))/(s*(s+7295)*(s+180.2)*(s^2+303.8*s+9.228*exp(04)))

%D=0.007343*(1+0.5*s)*(1+3.3*s)*(1+5.5*exp(1)-0.8*s+(0.0032*s)^2)/(s*(1+0.00059*s)*(1+3.9*exp(1)-0.5*s)*(1+0.0032*s+(0.0032*s)^2));

%D=0.0002833*(1+0.58*s)*(1+12*s)/(s*(1+0.00011*s)*(1+0.016*s)); %BEST ONE

%D=0.0002833*(1+0.58*s)*(1+12*s)/(s*(1+0.0014*s)*(1+0.016*s));

D=0.0002833*(1+0.63*s)*(1+35*s)/(s*(1+0.0033*s)*(1+0.0088*s)); %Upload

%margin(G*H*D) %open loop GM PM

margin((D*G*H)/(1+(D*G*H))) %closed loop r to y

%margin((G*H)/(1+D*G*H)) %e to y
% controlSystemDesigner(G*H,D)
