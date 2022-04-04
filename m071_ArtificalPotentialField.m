clear; clc; close all;

po = [1.2; 1.1];
Ro = 0.5;

pa = [0.0; 0.0];             % 目标位置
Ds  = 0.32;                  % 安全距离

step = 5e-2;
time = 10;
N  = time/step;

opt= optimset('Display','off');
M  = size(po, 2);

eta = 10;

xx=linspace(-1.0,3.0,50);
yy=linspace(-1.0,3.0,50);
[x,y]=meshgrid(xx,yy);                      % 平面
z=eta-10*((x-po(1,1)).^2+(y-po(2,1)).^2);
z(z<0) = 0;
figure(1);hold on;
surf(x,y,z, 'edgecolor', 'none','FaceAlpha',0.8);
xlabel('$x[m]$','interpreter','latex');                % x轴坐标 标题
ylabel('$y[m]$','interpreter','latex');                % x轴坐标 标题
zlabel('$z[m]$','interpreter','latex');                % x轴坐标 标题
set(gca, 'fontsize', 16);
set(gcf, 'color', 'w');
axis([-1,3,-1,3, -5, 10]);
view(214,40);
% view(-180,90);
pathHandle = animatedline('Linestyle', '-', 'color', 'r', 'Markersize', 2, 'Marker', 'none', 'Linewidth', 2);
pathHandle2 = animatedline('Linestyle', '-', 'color', 'r', 'Markersize', 2, 'Marker', 'none', 'Linewidth', 2);

v = zeros(2, 1);
xN = zeros(2, N);
xN(:, 1) =  [3;3];
agentHandle = plot3(pa(1, 1), pa(2, 1), 0, 'Marker', 'o', 'MarkerSize', 20, 'color', 'g','Linewidth',2); 
agentHandle2 = plot3(xN(1, 1), xN(2, 1), 1, 'Marker', 'o', 'MarkerSize', 3, 'color', 'r','Linewidth',5); 

for k = 2:N
    x_no = xN(:, k-1) - po;
    u = -(xN(:, k-1) - pa);
    V_c = max([eta-10*norm(x_no)^2, 0]);
    if(V_c > 0)
        dv= -(v - u) + 0.35*20*x_no;
    else
        dv= -(v - u);
    end
   
   
   v = v + dv*step;
   xN(:, k) = xN(:, k-1) + v *step;
%    addpoints(pathHandle, xN(1, k-1), xN(2, k-1), 1);
   addpoints(pathHandle2, xN(1, k-1), xN(2, k-1), V_c);
   
%    agentHandle.XData = xN(1, k-1);
%    agentHandle.YData = xN(2, k-1);
   agentHandle2.XData = xN(1, k-1);
   agentHandle2.YData = xN(2, k-1);
   agentHandle2.ZData = V_c+0.5;
   pause(0.05);
end

