K = 0.2;

s = tf('s');

G_s = K * (1 - s/0.142) / (s * (1 + s/0.325) * (1 + s/0.0362));

figure;
nyquist(G_s);
title('Nyquist Plot of the Transfer Function');