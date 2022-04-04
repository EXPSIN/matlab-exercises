clear; clc; close all;

figure(1); clf; axis equal; N = 200;
% init
for k = 1:N
    x(:, k) = 6*pi*[sin(k/N*(2*pi)); cos(k/N*(2*pi))];
    l_handle(k) = animatedline('color', [ones(1,3)*(k-1)/N*0.9]);
end

for t = 1:650
%     dx = [1 2; -1 1]*x;
%     dx = [0 1; -1 -1]*x;
%     dx = [0 0.5; -0.5 -1]*x;
    dx = [-x(1, :)+x(2, :); -5*sin(x(1, :))-9/29*x(2, :)];
    x  = x + dx*0.05;
    for k = 1:N
        addpoints(l_handle(k), x(1, k), x(2, k));
    end
    drawnow;
end