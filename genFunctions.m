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
syms tau1 tau2 % Torques.

q = [th1; th2]; % Vector of generalized coordinates.
dq = diff(q,t);
ddq = diff(dq,t);

tau = [tau1; tau2]; % Vector of non-conservative forces.

% Forward kinematics: center positions of each body.
r1 = [0.5*p.L1*cos(th1); 0.5*p.L1*sin(th1)]
r2 = [p.L1*cos(th1) + 0.5*p.L2*cos(th1+th2); 
    p.L1*sin(th1) + 0.5*p.L2*sin(th1+th2)]

% Velocities.
dr1 = diff(r1,t)
dr2 = diff(r2,t)

% Accelerations.
ddr1 = diff(dr1,t)
ddr2 = diff(dr2,t)

% Kinetic energy: 0.5*m*v (dot) v
T = 0.5*p.m1*dr1.'*dr1 + 0.5*p.m2*dr2.'*dr2
% Potential energy: m*g*h
V = p.m1*p.g*[0 1]*r1 + p.m2*p.g*[0 1]*r2

% Lagrangian
L = T- V


% Euler-Lagrange equation
dL_dq = diff(L,q)
dL_ddq = gradient(L,dq)
d_dt_dL_ddq = diff(dL_ddq,t)

EL_LHS = d_dt_dL_ddq - dL_dq;

B = [1 -1; 0 1]; % Input matrix.
EL_RHS = B*tau

% Put everything on the LHS.
EL = EL_LHS - EL_RHS

% Remove time dependence. Substitute symbolic functions with symbolic variables.

syms th1_ th2_ dth1_ dth2_ ddth1_ ddth2_ q_ dq_ ddq_

EL_ = subs(EL, diff(th1,t,t), ddth1_);
EL_ = subs(EL_, diff(th1,t), dth1_);
EL_ = subs(EL_, th1, th1_);

EL_ = subs(EL_, diff(th2,t,t), ddth2_);
EL_ = subs(EL_, diff(th2,t), dth2_);
EL_ = subs(EL_, th2, th2_)

% Get the equations of motion into the manipulator form, M*ddq = f.
