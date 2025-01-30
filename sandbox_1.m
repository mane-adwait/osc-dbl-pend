clear all
syms a;
b = [a;2*a];
disp(size(a));
disp(size(b));
syms t c(t);
d = [a;c];
disp(size(c));
disp(size(d));
disp(numel(d));

% Notes
% b is of type sym (symbolic variable), 
% not symfun (symbolic function)