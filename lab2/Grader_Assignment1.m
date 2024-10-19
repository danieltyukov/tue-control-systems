%% ************** SYSTEM PARAMETERS ****************************************
R    = 5;       % (ohm)
cL   = 1.9e-3;  % (m)
Ld   = 55e-3;   % (H)
m    = 0.018;   % (kg)
g    = 9.81;    % (m/s^2)
Linf = 0.1;     % (H)

% Non-Linear plant model
f = @(x1,x2,x3,u) [((u-R*x1+x3*x1*Ld*cL/((cL+x2)^2))/(Linf+Ld*cL/(cL+x2))),(x3),(g-(Ld*cL*(x1^2))/(2*m*((cL+x2)^2)))].';
    
% Operating point
Y0    = 10e-3; % Height of the ball in operating point
Ydot0 = 0;   % Speed of the ball in operating point
I0    = sqrt((2*g*m)/(Ld*cL))*(cL+Y0);   % Current in operating point
U0    = R*I0;   % Voltage in operating point
L0    = Linf+(Ld*cL)/(cL+Y0);   % Inductance in operating point
dL0dy = -(Ld*cL)/((cL+Y0)^2);   % Derivative of the Inductance in operating point wrt the position
x0    = [I0,Y0,Ydot0].';   % State of the system in the operating point
% .' means transposition of a matrix