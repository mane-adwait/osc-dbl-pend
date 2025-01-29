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

q = [th1; th2]; % Vector of generalized coordinates.

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