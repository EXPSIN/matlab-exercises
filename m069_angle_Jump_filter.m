clear all; close all; clc; 
N       = 1e3;
T       = 10e-3;
angle   = linspace(-2*pi,2*pi, N);
theta   = mod(angle+pi, 2*pi) - pi;          % real value of angle
t       = 0:T:(N-1)*T;
theta_s = mod(angle + 1e-1*randn(size(theta))+pi, 2*pi) - pi;  % sample value of angle
theta_f = 0;
res1    = zeros(size(theta));

sctheta = [0; 1];
res2    = zeros(size(theta));

theta_i_last = 0;
theta_i = 0;
theta_d = 0;
res3    = zeros(size(theta));

for idx = 1:N
    % 一阶惯性环节
    theta_f = rungekutta(@filter, theta_f, theta_s(idx), T);
    res1(idx) = theta_f;
    
    % 正反三角运算变换 再滤波
    sctheta  = rungekutta(@anglefilter, sctheta, theta_s(idx), T);    % y
    res2(idx) = atan2(sctheta(1), sctheta(2));  % x/y
        
    % 积分再微分
%     theta_i_last = theta_i;
%     theta_i      = theta_i + theta_s(idx)*T;     % 积分
%     theta_i      = rungekutta(@filter, theta_i, theta_i + theta_s(idx)*T, T);    % 积分
%     theta_d      = (theta_i-theta_i_last)/T;     % 再微分
%     res3(idx)    = theta_d;  % x/y
end

figure(1);  hold on;
% plot(t, theta_s,   'r', 'linewidth', 0.5, 'DisplayName','sample');
plot(t,   theta, 'k--', 'linewidth', 2.0, 'DisplayName','real');
plot(t,    res1,   'b', 'linewidth', 2.0, 'DisplayName','filter');
plot(t,    res2,   'm', 'linewidth', 2.0, 'DisplayName','filter2');
% plot(t, res3, 'k', 'linewidth', 2, 'DisplayName','i d');

legend; grid minor;
set(gca, 'fontsize', 24); 




function x = rungekutta(fun, x0, u, h)
% FcnHandlesUsed  = isa(fun,'function_handle');
k1 = fun(x0       , u);
k2 = fun(x0+h/2*k1, u);
k3 = fun(x0+h/2*k2, u);
k4 = fun(x0+  h*k3, u);
x = x0 + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function dx = filter(x, u)
Tf = 0.1;
dx = -1/Tf*(x-u);
end

function dx = anglefilter(x, u)
Tf = 0.1;
dx = -1/Tf*(x-[sin(u); cos(u)]);
end

