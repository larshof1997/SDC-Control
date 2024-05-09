% Vehicle parameters
m = 474; % mass (kg)
Iz = 339; % yaw moment of inertia (kg*m^2)
lf = 1.686/2; % distance from CG to front axle (m)
lr = 1.686/2; % distance from CG to rear axle (m)
cf = 1000; % front cornering stiffness (N/rad)
cr = 1000; % rear cornering stiffness (N/rad)

% Longitudinal parameters
rho = 1.225; % air density (kg/m^3)
A = 2.3; % frontal area (m^2)
Cd = 0.32; % drag coefficient
Cr = 0.01; % rolling resistance coefficient
g = 9.81; % gravitational acceleration (m/s^2)

% Simulation parameters
t_end = 10; % simulation end time (seconds)
dt = 0.01; % time step (seconds)
time = 0:dt:t_end; % time vector

% Sinusoidal steering input
steering_amplitude = 0.1; % radians (about 5.7 degrees)
steering_frequency = 1; % Hz
delta = steering_amplitude * sin(2 * pi * steering_frequency * time);

% Tractive force (constant for simplicity)
F_traction = 1000; % tractive force (N)

% Initialization
vy = zeros(size(time)); % lateral velocity (m/s)
r = zeros(size(time)); % yaw rate (rad/s)
vx = ones(size(time)) * 10; % longitudinal velocity (m/s), starting at 10 m/s
ax = zeros(size(time)); % longitudinal acceleration (m/s^2)
alpha_f = zeros(size(time)); % front slip angle (rad)
alpha_r = zeros(size(time)); % rear slip angle (rad)
Fyf = zeros(size(time)); % front lateral force (N)
Fyr = zeros(size(time)); % rear lateral force (N)

for i = 2:length(time)
    % Slip angles
    alpha_f(i) = delta(i) - atan((vy(i-1) + lf * r(i-1)) / max(vx(i-1), 0.1));
    alpha_r(i) = -atan((vy(i-1) - lr * r(i-1)) / max(vx(i-1), 0.1));
    
    % Lateral forces
    Fyf(i) = cf * alpha_f(i);
    Fyr(i) = cr * alpha_r(i);
    
    % Lateral dynamics
    ay = (Fyf(i) + Fyr(i)) / m;
    vy(i) = vy(i-1) + ay * dt;
    r(i) = r(i-1) + (lf * Fyf(i) - lr * Fyr(i)) / Iz * dt;
    
    % Longitudinal dynamics
    F_drag = 0.5 * rho * A * Cd * vx(i-1)^2;
    F_rolling = Cr * m * g;
    ax(i) = (F_traction - F_drag - F_rolling) / m;
    vx(i) = vx(i-1) + ax(i) * dt;
end

% Plotting the results to show how each parameter evolves over time and with changing steering inputs
figure;
subplot(4,2,1);
plot(time, delta);
title('Steering Angle vs Time');
xlabel('Time (s)');
ylabel('Steering Angle (rad)');

subplot(4,2,2);
plot(time, vx);
title('Longitudinal Velocity vs Time');
xlabel('Time (s)');
ylabel('Longitudinal Velocity (m/s)');

subplot(4,2,3);
plot(time, vy);
title('Lateral Velocity vs Time');
xlabel('Time (s)');
ylabel('Lateral Velocity (m/s)');

subplot(4,2,4);
plot(time, r);
title('Yaw Rate vs Time');
xlabel('Time (s)');
ylabel('Yaw Rate (rad/s)');

subplot(4,2,5);
plot(time, alpha_f, time, alpha_r);
title('Slip Angles vs Time');
xlabel('Time (s)');
ylabel('Slip Angle (rad)');
legend('Front', 'Rear');

subplot(4,2,6);
plot(time, Fyf, time, Fyr);
title('Lateral Forces vs Time');
xlabel('Time (s)');
ylabel('Lateral Force (N)');
legend('Front', 'Rear');

subplot(4,2,7);
plot(alpha_f, Fyf);
title('Front Lateral Force vs Front Slip Angle');
xlabel('Slip Angle (rad)');
ylabel('Lateral Force (N)');

subplot(4,2,8);
plot(alpha_r, Fyr);
title('Rear Lateral Force vs Rear Slip Angle');
xlabel('Slip Angle (rad)');
ylabel('Lateral Force (N)');

