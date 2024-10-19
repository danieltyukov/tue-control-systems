%% System parameters
R    = 5;       % (ohm)
cL   = 1.9e-3;  % (m)
Ld   = 55e-3;   % (H)
m    = 0.018;   % (kg)
g    = 9.81;    % (m/s^2)
Linf = 0.1;     % (H)
% operating point
Y0=10e-3;       % (m)

%% From Assignment 1
Ydot0 = 0;   % Speed of the ball in operating point
I0    = sqrt((2*g*m)/(Ld*cL))*(cL+Y0);   % Current in operating point
U0    = R*I0;   % Voltage in operating point
L0    = Linf+(Ld*cL)/(cL+Y0);   % Inductance in operating point
dL0dy = -(Ld*cL)/((cL+Y0)^2);   % Derivative of the Inductance in operating point wrt the position

%% From Assignment 2
% Give the state space matrices such that
%       d/dt x = Ax + Bu
%            y = Cx + Du
%A = [-R,0,0;0,0,1;(g-Ld*cL*I0/(m*((cL+Y0)^2))),(g+Ld*cL*(I0^2)/(m*((cL+Y0)^3))),0];
%A = [-R,0,0;0,0,1;(-Ld*cL*I0/(m*((cL+Y0)^2))),(Ld*cL*(I0^2)/(m*((cL+Y0)^3))),0];
%A = [-R/L0,0,0;0,0,1;(-Ld*cL*I0/(m*((cL+Y0)^2))),(Ld*cL*(I0^2)/(m*((cL+Y0)^3))),0];
A = [-R/L0,0,(Ld*cL*I0)/(L0*((cL+Y0)^2));0,0,1;(-Ld*cL*I0/(m*((cL+Y0)^2))),(Ld*cL*(I0^2)/(m*((cL+Y0)^3))),0];
B = [1/L0,0,0].';
C = [0,1,0];
D = 0;

% Give the linear transfer function
s = tf('s');    % Laplace variable s
G = C * ((s * eye(size(A)) - A) \ B) + D;
%G = -260.7/(s^3+45.96*(s^2)-1516*s-75780);

G = minreal(G);             % To deal with pole-zero cancelations when comparing to the solution
sys = ss(A,B,C,D);          % Linearized state space
%G2=tf(sys)
[yss,tss] = step(sys,0.1);  % step response for state space
[ytf,ttf] = step(G,0.1);    % step response for transfer function
figure()
plot(tss,yss+Y0,ttf,ytf+Y0,'--','linewidth',2)
xlabel('Time [s]')
ylabel('Height [m]')
grid on
legend('State space','Transfer function')
axis([0 0.07 0 12e-3])


