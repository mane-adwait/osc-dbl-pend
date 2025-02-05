% mainSim.m
% Simulate the uncontrolled dynamics.

clc; clear; close all;

odefun = @(t,x) dynamics(t,x,[0;0]);

% State: x = [th1; dth1; th2; dth2]
[tout, xout] = ode45(odefun, [0 5], [-pi/2; 0; 0.1; 0]) ;

plot(tout,xout(:,1)); xlabel('Time (s)'); ylabel('Angle (rad)');