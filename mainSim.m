% mainSim.m
% Simulate the controlled system.

clc; clear; close all;

t_span = [0 1]; % Total duration (s).
% Control timestep: At each control timestep, the state is sampled, 
% the QP is run, and the control input is computed.
dt = 0.05; % Control timestep (s).
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
    
    tau = [w_star(1); w_star(2)];

    odefun = @(t,x) dynamics(t,x,tau);

    % Quadratic program: decision vector, cost function, constraints.

    [t_col, x_col] = ode45(odefun, sim_t_span, x0_current) ;
    % Time is the vertical axis by default in ode45 output, hence "_col" 
    % in the variable names. I prefer time to be the horizontal axis, so I 
    % take the transpose. 
    t_out = t_col.'; x_out = x_col.'; 
    x0_current = x_out(:,end); 
    
    % Append the results to the previous results.
    t_store = [t_store t_out];
    x_store = [x_store x_out];
end

fig1 = figure; movegui(fig1,'northeast');
plot(t_store,x_store(1,:)); xlabel('Time (s)'); ylabel('Angle (rad)');

%% Animation

FPS = 40; % Frames per second.
t_anim = t_span(1):1/FPS:t_span(2);

% interp1 requires the sample points (t_store) to be unique, but there are 
% duplicate values in t_store. This often happens when ode45 outputs 
% repeated time points due to adaptive time stepping.
% The "unique" command gives the indices and values of the unique values.
[t_store_unique, idx] = unique(t_store, 'stable'); 
x_store_unique = x_store(:, idx);

x_anim_col = interp1(t_store_unique.', x_store_unique.', t_anim.');
x_anim = x_anim_col.';

% Set up the video writer object.
video_filename = 'animation.mp4';
video_writer = VideoWriter(video_filename, 'MPEG-4');
video_writer.FrameRate = FPS;
open(video_writer);

fig2 = figure; movegui(fig2,"north");
for iter = 1:numel(t_anim)
    cla; hold on;
    th1 = x_anim(1,iter); th2 = x_anim(3,iter);
    plot([0, cos(th1), cos(th1)+cos(th1+th2)], ...
        [0, sin(th1), sin(th1)+sin(th1+th2)], ...
        'k-', 'LineWidth',3 ...
        )
    axis equal; 
    axis([-3 3 -3 3]);
    drawnow

    % Capture the frame and write it to the video writer object.
    frame = getframe(fig2);
    writeVideo(video_writer, frame);    
end

% Close the video writer object.
close(video_writer);
fprintf('Animation saved as %s\n', video_filename);


%%
% Resume CH lecture video at 0:15:00.