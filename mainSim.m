% mainSim.m
% Simulate the uncontrolled dynamics.

clc; clear; close all;

odefun = @(t,x) dynamics(t,x,[0;0]);

t_span = [0 5]; % Total duration (s).
dt = 0.01; % Sampling time (s). The state is sampled and the QP is run once each interval.
% Control rate = 1/dt (Hz).
t_vector = t_span(1):dt:t_span(2);

% State: x = [th1; dth1; th2; dth2]
x0 = [-pi/2; 0; 0.1; 0]; % Initial state.
x0_current = x0;

% Simulation loop.
for iter = 1:numel(t_vector)-1
    sim_t_span = [t_vector(iter) t_vector(iter+1)];

    [tout, xout] = ode45(odefun, sim_t_span, x0_current) ;
end

plot(tout,xout(:,1)); xlabel('Time (s)'); ylabel('Angle (rad)');

% Resume video at 28:00.