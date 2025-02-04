function [dx] = dynamics(~,x,tau)

% State: x = [th1; dth1; th2; dth2]

% Unpack the state vector.
q = [x(1); x(3)]; dq = [x(2); x(4)];

ddq = Mfunc(q,dq,tau)^-1 * ffunc(q,dq,tau);

dx = [x(2); ddq(1); x(4); ddq(2)];
