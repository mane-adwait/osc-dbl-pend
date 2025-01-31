clc; clear all

fprintf(['Takeaway: size and numel work for symbolic variables (type sym), ' ...
    '\n not symbolic functions (type symfun) \n\n'])

fprintf(['Use matrix mutiplication for indexing, ' ...
    'e.g [1 0]*d to index the first value of d, [0 1]*d to index the second'])

syms a;
b = [a; 2*a];
disp(['size(a) = ', num2str(size(a))]);
disp(['numel(a) = ', num2str(numel(a))]);
disp(['size(b) = ', num2str(size(b))]);
disp(['numel(b) = ', num2str(numel(b))]);
fprintf('\n')

syms t c(t);
d = [a;c];
disp(['size(c) = ', num2str(size(c))]);
disp(['numel(c) = ', num2str(numel(c))]);
disp(['size(d) = ', num2str(size(d))]);
disp(['numel(d) = ', num2str(numel(d))]);

% Appendix.
% disp(['size(??) = ', num2str(size(_))]);
% disp(['numel(??) = ', num2str(numel(_))]);