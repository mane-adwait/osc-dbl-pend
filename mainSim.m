% mainSim.m
% Simulate the uncontrolled dynamics.

clc; clear; close all;

odefun = @(t,x) dynamics(t,x,[0;0]);

t_span = [0 5]; % Total duration (s).
% Control timestep:
% At the start of each interval, the state is sampled, the QP is run, and 
% the control torque is computed.
dt = 0.01; % Sampling time interval (s). 
% Control rate = 1/dt (Hz).
t_vector = t_span(1):dt:t_span(2);

% State: x = [th1; dth1; th2; dth2]
x0 = [-pi/2; 0; 0.1; 0]; % Initial state.
x0_current = x0;

% Store the time and state output.
t_store = []; x_store = [];

% Simulation loop.
for iter = 1:numel(t_vector)-1
    sim_t_span = [t_vector(iter) t_vector(iter+1)];
    
    % We need to compute the control input and apply to the dynamics 
    % at each loop iteration.

    [tout, xout] = ode45(odefun, sim_t_span, x0_current) ;
    x0_current = xout(end,:).'; 
    
    % Append the results to the previous results.
    t_store = [t_store tout.'];
    x_store = [x_store xout.'];
end

plot(t_store,x_store(1,:)); xlabel('Time (s)'); ylabel('Angle (rad)');

% Resume video at ??.