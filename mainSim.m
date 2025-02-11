% mainSim.m
% Simulate the uncontrolled dynamics.

clc; clear; close all;

odefun = @(t,x) dynamics(t,x,[0;0]);

t_span = [0 1]; % Total duration (s).
dt = 0.1; % Sampling time interval (s). The state is sampled and the QP is run once each interval.
% Control rate = 1/dt (Hz).
t_vector = t_span(1):dt:t_span(2);

% Initialize the output vectors. We will append the simulation outputs to
% these vectors.
t_out = t_span(1); 
% State: x = [th1; dth1; th2; dth2].
x_out = [-pi/2; 0; 0.1; 0]; % Initial state.
% x_vector = zeros(4,numel(t_vector));
% x_vector(:,1) = [-pi/2; 0; 0.1; 0]; % State: x = [th1; dth1; th2; dth2].

% Simulation loop.
for iter = 1:numel(t_vector)-1
    sim_t_span = [t_vector(iter) t_vector(iter+1)];

    [t_iter_out, x_iter_out] = ode45(odefun, sim_t_span, x_vector(:,iter) ) ;
    
    t_out = [t_out t_iter_out];
    x_out = [x_out x_iter_out];

    % Interpolate tout and xout, and replace the preallocated x_vector with
    % the interpolated values.
    % vq = interp1(x,v,xq)
end

plot(t_out,x_out(:,1)); xlabel('Time (s)'); ylabel('Angle (rad)');