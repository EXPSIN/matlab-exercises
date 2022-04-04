clear; clc; close all;

figure(1); clf; axis equal;hold on; N = 100;
% init
for k = 1:N
%     if(mod(k, 5) == 1)
%         x(:, k) = 0.1*5*[sin(k/N*(2*pi)); cos(k/N*(2*pi))];
%     else
%         x(:, k) = 5*[sin(k/N*(2*pi)); cos(k/N*(2*pi))];
%     end
%     x(:, k) = 5*[sin(k/N*(2*pi)); cos(k/N*(2*pi))];

    h_cnt = ceil(sqrt(N));
    row_idx = ceil(k/h_cnt);
    col_idx = mod(k, h_cnt);
    x(:, k) = 10*[(col_idx - h_cnt / 2)/h_cnt*2; (-row_idx + h_cnt / 2)/h_cnt*2];
    l_handle(k) = animatedline('color', [ones(1,3)*(k-1)/N*0.9]);
    addpoints(l_handle(k), x(1, k), x(2, k));
end
handle_now = line(x(1, :), x(2, :), 'Marker', 'o', 'linestyle', 'none', 'color','r');
axis([-8 8 -8 8]); axis equal;

epsilon = 1;

for t = 1:100
    for k = 1:N
        % Q2.8
%         dx = [ x(2, k);...
%               -x(1, k)+1/16*(x(1, k)^5)-x(2, k)];
        % Q2.5
%         dx = [-x(1, k) - x(2, k)/log(norm(x(:,k))); ...
%               -x(2, k) + x(1, k)/log(norm(x(:,k)))];
        % Q2.4(2)
        dx = [ 2*x(1, k)-x(1, k)*x(2, k);...
              2*x(1, k)^2-x(2, k)];
        if(norm(x(:, k)) > 1e2)
            dx = zeros(size(dx));
        end
        x(:, k) = x(:, k) + dx*0.05;
        addpoints(l_handle(k), x(1, k), x(2, k));
    end
    handle_now.XData = x(1, :);
    handle_now.YData = x(2, :);
    drawnow;
end