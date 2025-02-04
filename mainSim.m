% mainSim.m
% Simulate the uncontrolled dynamics.

clc; clear; close all;

odefun = @(t,x) dynamics(t,x,[0;0]);

[tout, xout] = ode45(odefun, [0 5], [pi/4; 0; pi/4; 0]) ;

plot(tout,xout(:,1))