% genFunctions.m
% Adwait Mane, 2025 Jan 17

% Generate the required functions for Operational Space Control (OSC) of a
% double pendulum.
% - Forward dynamics (M, f).
% - Task space kinematics and derivatives (y, dy, ddy, J, Jdot).
% - Desired task-space acceleration (PD controller).
% - Optimization: QP (Q, c, A, b, Aeq, beq)

% Clear everything.
clc; clear; close all;

% Generate our dynamics.
p = getParams(); % Store the parameters.

% Lagrangian dynamics.
% Generalized coordinates.

syms t % Time.
syms th1(t) th2(t) % Joint angles.
syms tau1 tau2 % Input torques.

q = [th1; th2]; % Vector of generalized coordinates.
dq = diff(q,t); % Velocities.
ddq = diff(dq,t); % Accelerations.

tau = [tau1; tau2]; % Vector of non-conservative forces.

% Forward kinematics to define the task space: position of the
% end-effector.
y = [p.L1*cos(th1) + p.L2*cos(th1+th2); 
    p.L1*sin(th1) + p.L2*sin(th1+th2)];
dy = diff(y,t); % Velocity.
ddy = diff(dy,t); % Acceleration.

% Forward kinematics: center positions of each body.
r1 = [0.5*p.L1*cos(th1); 0.5*p.L1*sin(th1)];
r2 = [p.L1*cos(th1) + 0.5*p.L2*cos(th1+th2); 
    p.L1*sin(th1) + 0.5*p.L2*sin(th1+th2)];

% Velocities of body centers.
dr1 = diff(r1,t);
dr2 = diff(r2,t);

% Accelerations of body centers.
ddr1 = diff(dr1,t);
ddr2 = diff(dr2,t);

% Kinetic energy: 0.5*m*v (dot) v
T = 0.5*p.m1*dr1.'*dr1 + 0.5*p.m2*dr2.'*dr2 ;
% Potential energy: m*g*h
V = p.m1*p.g*[0 1]*r1 + p.m2*p.g*[0 1]*r2 ;

% Lagrangian
L = T - V ;


% Euler-Lagrange equation
dL_dq = diff(L,q) ;
dL_ddq = gradient(L,dq) ;
d_dt_dL_ddq = diff(dL_ddq,t) ;

EL_LHS = d_dt_dL_ddq - dL_dq;

B = [1 -1; 0 1]; % Input matrix.
EL_RHS = B*tau;

% Put everything on the LHS.
EL = EL_LHS - EL_RHS;

% Remove time dependence. Substitute symbolic functions with symbolic variables.
syms th1_ th2_ dth1_ dth2_ ddth1_ ddth2_ q_ dq_ ddq_

q_ = [th1_; th2_]
dq_ = [dth1_; dth2_]
ddq_ = [ddth1_; ddth2_]

EL_temp = subs(EL, diff(th1,t,t), ddth1_);
EL_temp = subs(EL_temp, diff(th1,t), dth1_);
EL_temp = subs(EL_temp, th1, th1_);

EL_temp = subs(EL_temp, diff(th2,t,t), ddth2_);
EL_temp = subs(EL_temp, diff(th2,t), dth2_);
EL_temp = subs(EL_temp, th2, th2_);

% EL_temp is still of type symfun, even though we removed the time
% dependence of the terms via substitution. So we create EL_ below.
EL_(1,1) = [1 0]*EL_temp; EL_(2,1) = [0 1]*EL_temp; 
disp(['EL_ = ' newline char([1 0] * EL_) newline char([0 1] * EL_)])

% Repeat for the task space, y.
y_temp = subs(y, diff(th1,t,t), ddth1_);
y_temp = subs(y_temp, diff(th1,t), dth1_);
y_temp = subs(y_temp, th1, th1_);

y_temp = subs(y_temp, diff(th2,t,t), ddth2_);
y_temp = subs(y_temp, diff(th2,t), dth2_);
y_temp = subs(y_temp, th2, th2_);

y_(1,1) = [1 0]*y_temp; y_(2,1) = [0 1]*y_temp; 
disp(['y_ = ' newline char([1 0] * y_) newline char([0 1] * y_)])

% Get the equations of motion into the manipulator form, M*ddq = f.
M = jacobian(EL_,ddq_); % Inertia matrix.
f = -subs(EL_, ddq_, [0;0]);

% Export these functions.
matlabFunction(M,'File','Mfunc','Vars',{q_,dq_,tau});
matlabFunction(f,'File','ffunc','Vars',{q_,dq_,tau});

% ------ APPENDIX -------
% disp(['var = ' newline char([1 0] * var) newline char([0 1] * var)])

% ------ SANDBOX -------
% fprintf('EL_ = \n%s\n%s\n', char([1 0]*EL_), char([0 1]*EL_));
% disp(['EL_ = \n' char(EL_)]) % Does not work.