clear; clc; close all;

figure(1); clf; axis equal;hold on; N = 100;
% init
for k = 1:N
    if(mod(k, 5) == 1)
        x(:, k) = 0.1*5*[sin(k/N*(2*pi)); cos(k/N*(2*pi))];
    else
        x(:, k) = 5*[sin(k/N*(2*pi)); cos(k/N*(2*pi))];
    end
    
    l_handle(k) = animatedline('color', [ones(1,3)*(k-1)/N*0.9]);
end
handle_now = line(x(1, :), x(2, :), 'Marker', 'o', 'linestyle', 'none', 'color','r');

epsilon = 1;


for t = 1:500   
    for k = 1:N
        dx = [x(2, k); -x(1, k)+epsilon*(1-x(1, k)^2)*x(2, k)];
        x(:, k) = x(:, k) + dx*0.05;
        handle_now.XData = x(1, :);
        handle_now.YData = x(2, :);
        addpoints(l_handle(k), x(1, k), x(2, k));
    end
    drawnow;
end