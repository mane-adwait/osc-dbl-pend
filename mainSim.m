% mainSim.m
% Simulate the controlled system.

clc; clear; close all;

t_span = [0 5]; % Total duration (s).
% Control timestep: At each control timestep, the state is sampled, 
% the QP is run, and the control input is computed.
dt = 0.01; % Control timestep (s).
% Control frequency = 1/dt (Hz).
t_vector = t_span(1):dt:t_span(2);

% State: x = [th1; dth1; th2; dth2]
x0 = [-pi/2; 0; 0.1; 0]; % Initial state.
x0_current = x0;

% Controller gains.
Kp = 10; Kd = 1;

% Desired position in the task space.
y_des = [0.5; 0.5]; dy_des = [0; 0];

% Initialize variables to store the time and state outputs.
t_store = []; x_store = [];

% Simulation loop.
for iter = 1:numel(t_vector)-1
    sim_t_span = [t_vector(iter) t_vector(iter+1)];
    
    % Unpack the state to measure q and dq.
    q = [x0_current(1); x0_current(3)]; dq = [x0_current(2); x0_current(4)];

    % Define the desired acceleration in the task space.
    ddy_des = Kp*(y_des-yfunc(q)) + Kd*(dy_des-dyfunc(q,dq));
    
    % We need to compute the control input and apply it to the dynamics 
    % at each loop iteration.
    
    % Solve the QP for Operational Space Control.
    w_star = quadprog(...
        Qfunc(q,dq,ddy_des), ...
        cfunc(q,dq,ddy_des), ...
        [],[], ...
        Aeqfunc(q,dq), ...
        beqfunc(q,dq) ...
        );
    
    odefun = @(t,x) dynamics(t,x,tau);

    % Quadratic program: decision vector, cost function, constraints.

    [tout, xout] = ode45(odefun, sim_t_span, x0_current) ;
    x0_current = xout(end,:).'; 
    
    % Append the results to the previous results.
    t_store = [t_store tout.'];
    x_store = [x_store xout.'];
end

plot(t_store,x_store(1,:)); xlabel('Time (s)'); ylabel('Angle (rad)');

% Resume video at 38:30.