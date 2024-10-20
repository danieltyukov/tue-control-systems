%% Stability 1
s = tf('s');
G = 1 / (s * (1 + s/5) * (1 + s/20));
C = 1;
L = C * G;
S = 1 / (1 + L);
T = L / (1 + L);

bode(S, 'r', T, 'b', L, 'g');
legend('Sensitivity', 'Complementary Sensitivity', 'Loop Transfer');

%% Stability 2
K = 40;
D = K * ((1+(s/5)) / (1+(s/50)));
L = D * G;
S = 1 / (1 + L);
T = L / (1 + L);

bode(S, 'r', T, 'b', L, 'g');
legend('Sensitivity', 'Complementary Sensitivity', 'Loop Transfer');