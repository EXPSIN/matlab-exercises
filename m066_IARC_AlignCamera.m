clear; clc; close all;

d    = 2;           % 距离
x    = [1; 0; 2*pi/3];  % [x轴；y轴；角度]
% x    = [-1; 0; pi/2];  % [x轴；y轴；角度]
u    = zeros(3,1);      % 机体速度(x,y) 期望偏航角度(yaw_r)
T    = 100e-3;          % 10ms 控制一次
FOV  = 2*pi/3;      % 相机视场角
N    = 10e2;        % 迭代次数
Time = N*T;         % 总时间 sec
panelData = [linspace(-0.5,0.5, 100); linspace(1.5, 1, 100)];

% 绘制图形
H = myGraphics([], x, panelData, FOV);

for k = 1:N
    t = k*T;    % 当前时间
    % 控制器
    samplePanelData = panelData + 5e-2*randn(size(panelData));
    u = controller(x, samplePanelData, FOV);
    
    % 求解
    [x, dx] = rungekutta(@uav_2Dmodel, x, u, T);
    
    % 更新图形
    H = myGraphics(H, x, samplePanelData, FOV);
%     pause(0.2);
    drawnow;
%     drawnow limitrate;
end

%{
    theta:  输入范围 0~2*pi
    theta_r: 输出范围 0~2*pi
%}
function err = angleErr(theta, theta_r)
err = theta_r - theta;
if(abs(err) > pi)
    err = 2*pi - sign(err)*err;
end 
end

function u = controller(x, panel, FOV)
uavTheta = x(3, 1);
uavPos   = x(1:2, 1);
% pDet     = panel-uavPos;
% camTheta = atan2(pDet(2, :), pDet(1, :));
% inCamIndex = acos(pDet'*[cos(uavTheta); sin(uavTheta)]./vecnorm(pDet) ) <= FOV/2;
panelCenter = mean(panel, 2);
L = norm(panelCenter-uavPos);
[panelDir, panelDirLength] = getLines(panel);

theta_s  = sign(cross([cos(uavTheta); sin(uavTheta); 0], [panelDir; 0]));
theta    = theta_s(3)*acos(alimit( [cos(uavTheta); sin(uavTheta)]'*panelDir ));
gamma_s  = sign(cross([cos(uavTheta); sin(uavTheta); 0], [panelCenter-uavPos; 0]));
gamma    = gamma_s(3)*acos(alimit( [cos(uavTheta); sin(uavTheta)]'*(panelCenter-uavPos)/norm(panelCenter-uavPos) ));
D = L*sin(gamma);

alpha = theta - gamma;
xn = -L*cos(alpha);
yn =  L*sin(alpha);
ux = (-1.0-xn)*1.0;
uy = (0-yn);

% 理想的
u(1:2, 1) = [cos(theta), -sin(theta); sin(theta), cos(theta)] * [ux; uy];
u(3) = uavTheta + theta;

% 目前在用的
u(1) = -1.0 + L;
u(2) = 0 - theta;
% u(1:2, 1) = [cos(theta), -sin(theta); sin(theta), cos(theta)] * [-1.0 + L; -theta];
u(3) = uavTheta + gamma;
end

function [dir, len] = getLines(points)
N = size(points, 2);
dir = (points*points')\points*ones(N, 1);
len = norm(dir);
dir = dir/len;     
len = 1/len;    % dir'*points = len;
end

function dx = uav_2Dmodel(x, u)
vx = u(1);          % 机体速度-x
vy = u(2);          % 机体速度-y
theta_r = u(3);     % 角度 给定
theta   = x(3, 1);  % 角度

dx = zeros(3, 1);
dx(1:2, 1) = [cos(theta), -sin(theta); sin(theta), cos(theta)] * [vx; vy];
dx(3, 1) = 1/1*(theta_r-theta);
end

% 龙格库塔
function [x, dx] = rungekutta(fun, x0, u, h)
k1 = fun(x0       , u);
k2 = fun(x0+h/2*k1, u);
k3 = fun(x0+h/2*k2, u);
k4 = fun(x0+  h*k3, u);
dx = (k1 + 2*k2 + 2*k3 + k4)/6;
x = x0 + h*dx + randn(3, 1)*0e-3;
x(3) = mod(x(3), 2*pi);
end

% handle xR xL {L, parking_space}
function handle = myGraphics(handle, x, panelData, FOV)
R = 5;  % 最大探测距离
if(isempty(handle))
%     figure(1); set(gcf, 'color', 'w', 'position', [100, 100, 800, 300]);
    figure(1); 
    hold on;
    grid on; 
    axis equal;
    set(gcf, 'color', 'w', 'position', [100, 100, 800, 600]);
    axis([-1,1,-0.5,2]);
    
    
    
    % 绘制飞行器的方向
    handle.tri_data = 0.1*[[1; 0], [-1/2; sqrt(3)/4], [0; 0], [-1/2; -sqrt(3)/4]];
    dir_mat         = rotate_mat2D(x(3))*handle.tri_data;
    handle.uav      = patch(x(1)+dir_mat(1, :), x(2)+dir_mat(2, :),'b', 'HandleVisibility', 'off');
    camTheta        = linspace(x(3)-FOV/2, x(3)+FOV/2, 15);
    camAarea        = [x(1)+[0, R*cos(camTheta), 0]; x(2)+[0, R*sin(camTheta), 0]];
    handle.cam      = patch(camAarea(1, :), camAarea(2, :),'c', 'HandleVisibility', 'off', 'facealpha', 0.1);
    
    
    % 绘制飞行器的轨迹
    handle.panelCenter = mean(panelData, 2);
    handle.panel       = plot(panelData(1, :), panelData(2, :), 'r', 'linestyle', '-', 'linewidth', 2);
    handle.panelC      = plot(handle.panelCenter(1), handle.panelCenter(2), 'k.', 'markersize', 30, 'linestyle', 'none', 'HandleVisibility', 'off');
    handle.path     = animatedline('color', 'b');
    
    legend('板面', '飞行器轨迹','Location', 'eastoutside');
    set(gca, 'fontsize', 20);
    
%     figure(2);
%     set(gcf, 'color', 'w', 'position', [650, 100, 800, 600]);
%     
%     hold on; grid minor;
%     
%     % 李雅普诺夫函数
%     handle.V       = animatedline('color', 'r');
%     handle.theta   = animatedline('color', 'b');
%     handle.theta_r = animatedline('color', 'b', 'linestyle', '--');
%     handle.v = animatedline('color', 'c', 'linestyle', '-');
%     handle.w = animatedline('color', 'k', 'linestyle', '-');
%     legend('V', '\theta', '\theta_r', '速度 v', '角速度 w', 'Location', 'eastoutside');
%     set(gca, 'fontsize', 20);
    return;
end


handle.panel.XData = panelData(1, :);
handle.panel.YData = panelData(2, :);
handle.panelC.XData = mean(panelData(1, :));
handle.panelC.YData = mean(panelData(2, :));

% % 添加轨迹
addpoints(handle.path, x(1), x(2));

% 改变方向
dir_mat         = rotate_mat2D(x(3))*handle.tri_data;
handle.uav.XData = x(1)+dir_mat(1, :);
handle.uav.YData = x(2)+dir_mat(2, :);

% 
camTheta    = linspace(x(3)-FOV/2, x(3)+FOV/2, 15); 
camAarea    = [x(1)+[0, R*cos(camTheta), 0]; x(2)+[0, R*sin(camTheta), 0]];
handle.cam.XData = camAarea(1, :);
handle.cam.YData = camAarea(2, :);
end

function R_mat = rotate_mat2D(yaw)
cYaw = cos(yaw);
sYaw = sin(yaw);
R_mat = [cYaw, -sYaw; sYaw, cYaw];
end

function number = alimit(number)
number = min(max(number, -1), 1);
end