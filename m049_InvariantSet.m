close all; clear; clc;
[xS, yS, zS] = sphere(6);       % 球形数据
sphereRatio = 0.03; xS = sphereRatio*xS; yS = sphereRatio*yS; zS = sphereRatio*zS;
[X,Y,Z] = cylinder([0 sqrt(3)/3], 30);   % 圆锥数据
x_new = 0; y_new = sqrt(3)/3; z_new = 1;
% x_new = 0; y_new = 0; z_new = 0;
idx = 1;
figure(1); set(gcf, 'position', [0+0 0 1280 600]);
set(gcf, 'Color', 'w');
%% ==============================
subplot(1,2,2); hold on;
contour(X,Y,Z);
title('俯视图'); set(gca, 'FontSize', 16);
xlabel('x'); ylabel('y'); axis equal; grid minor;
handle2DLine = animatedline('Color', 'r', 'LineStyle', '--');
circle(x_new, y_new, sphereRatio, 'b');

subplot(1,2,1); hold on;
mesh(X,Y,Z, 'FaceColor', 'interp', 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'EdgeAlpha', 0, 'LineStyle', '-');
colormap gray;
title('三维视图'); view(-45, 20); set(gca, 'FontSize', 16);
xlabel('x'); ylabel('y'); zlabel('z'); axis equal; grid minor;
handle3DLine = animatedline('Color', 'r', 'LineStyle', '--');
surf(x_new+xS, y_new+yS, z_new+zS, 'FaceColor', 'b','FaceAlpha',0.5,'EdgeColor', 'b', 'EdgeAlpha',1, 'LineStyle', '-');

%% 仿真数据
handleLast = [];
deltaT = 0.05;
vHorizontal = 1; vVertical = 1e-10;

while(z_new > 0)
    x = z_new / sqrt(3) * x_new / sqrt(x_new^2 + y_new^2);
    y = z_new / sqrt(3) * y_new / sqrt(x_new^2 + y_new^2);
    z = z_new;
    delete(handleLast);
    subplot(1,2,1); 
    handle3D = surf(x+xS, y+yS, z+zS, 'FaceColor', 'r','FaceAlpha',0.5,'EdgeColor', 'r', 'EdgeAlpha',1, 'LineStyle', '-');
    handle = legend('$O$', '$x(x_0, t)$', '$x_0$', '$x_{now}$');
    set(handle,'Interpreter','latex', 'FontSize', 12);
    subplot(1,2,2); handle2D = circle(x, y, sphereRatio, 'r');
    
%     handle = legend('$O$', '$x(t, x_0)$', '$x_0$');
%     set(handle,'Interpreter','latex', 'FontSize', 12);
    
    addpoints(handle2DLine, x, y);
    addpoints(handle3DLine, x, y, z);
    handleLast(1) = handle3D; handleLast(2) = handle2D;
    x_new =  deltaT * vHorizontal * y / sqrt(x^2 + y^2) + x;    % 谨慎使用欧拉法仿真，建议龙格库塔。
    y_new = -deltaT * vHorizontal * x / sqrt(x^2 + y^2) + y;
    z_new = -deltaT * vVertical * sqrt(3) / 2 + z;
    vVertical = sqrt(vVertical^2 + 2*1*(z - z_new));
    idx = idx + 1;
    
    drawnow;
end


%% ==============================
function handle2D = circle( x,y,r,c )
    handle2D = patch(x+r*cos(linspace(0, 2*pi, 12)), y+r*sin(linspace(0, 2*pi, 12)), c);
    
end


