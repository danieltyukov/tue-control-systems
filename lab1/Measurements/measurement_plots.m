%clear all

%step responce with disturbance
load('step_const_longer2.mat')
omega=step_const_longer2.Y(3).Data;
time=step_const_longer2.X(1).Data;
omega_ref=step_const_longer2.Y(4).Data;

omega_sim=Step_const_dist.signals(1).values;
time_sim=Step_const_dist.time;
omega_ref_sim=Step_const_dist.signals(2).values;

figure(1)
plot(time, omega_ref)
hold on
plot(time, omega)
hold on
plot(time_sim-1.3515, omega_ref_sim)
hold on
plot(time_sim-1.3515, omega_sim)
legend('reference input measurements', 'tracking input measurements', 'reference input simulation', 'tracking input simulation')
xlabel('Time (seconds)')
ylabel('Speed (rad/s)')


%step response with noise
load('step_sin2.mat')
omega1=step_sin2.Y(3).Data;
time1=step_sin2.X(1).Data;
omega_ref1=step_sin2.Y(4).Data;

omega1_sim=Step_sin_dist.signals(1).values;
time1_sim=Step_sin_dist.time;
omega_ref1_sim=Step_sin_dist.signals(2).values;


figure(2)
plot(time1, omega_ref1)
hold on
plot(time1, omega1)
hold on
plot(time1_sim-3.36882, omega_ref1_sim)
hold on
plot(time1_sim-3.36882, omega1_sim)
xlabel('Time (seconds)')
ylabel('Speed (rad/s)')
legend('reference input measurements', 'tracking input measurements', 'reference input simulation', 'tracking input simulation')

%square response with no noise
load('square_nodist2.mat')
omega2=square_nodist2.Y(3).Data;
time2=square_nodist2.X(1).Data;
omega_ref2=square_nodist2.Y(4).Data;

figure(3)
plot(time2, omega_ref2)
hold on
plot(time2, omega2)
xlabel('Time (seconds)')
ylabel('Speed (rad/s)')
legend('reference input', 'tracking input')

%square response with noise
load('square_sin2.mat')
omega3=square_sin2.Y(3).Data;
time3=square_sin2.X(1).Data;
omega_ref3=square_sin2.Y(4).Data;

figure(4)
plot(time3, omega_ref3)
hold on
plot(time3, omega3)
xlabel('Time (seconds)')
ylabel('Speed (rad/s)')
legend('reference input', 'tracking input')