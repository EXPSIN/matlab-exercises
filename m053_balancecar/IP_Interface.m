function GS = IP_Interface(varargin)

if(varargin{1} == 'i')          % initialization
    GS = GraphIntialization(varargin{2}, varargin{3});
elseif(varargin{1} == 'u')      % update
    GS = updateSystemModel(varargin{2});
    GS = GraphUpdate(GS);
elseif(varargin{1} == 's')      % debug graph
    GS = GraphUpdate(varargin{2});
else
    error("Unknow command %c", varargin{1});
end

end

%% Intialization the graph
function [GS, state] = GraphIntialization(T, theta0)
% Setting
GS.car.size = [0.1; 0.1; 0.1];  % meter
GS.car.mass = 1;                % kilogram
GS.car.color= 'red';          
GS.car.bias  = [0; 0; 0.1];
GS.car.p     = GS.car.bias(1);
GS.car.v     = 0;
GS.car.a     = 0;
GS.car.u     = 0;               % 电压设定值
GS.car.c0    = 0.05;           % 导轨摩擦系数
GS.g         = 9.8;

GS.pen.len  = 0.5;         % meter
GS.pen.R    = 0.025;        % meter
GS.pen.mass = 0.1;         % kilgram
GS.pen.N    =   5;         % 分割线数目
GS.pen.color = 'blue';     %
GS.pen.bias  = GS.car.bias + [GS.pen.R; 0; GS.pen.R];
GS.pen.theta = theta0;
GS.pen.thetav= 0;
GS.pen.thetaa= 0;
GS.pen.c0    = 0.05;      % 摆杆摩擦力矩
GS.pen.J     = (GS.pen.len)^2 * GS.pen.mass;  % 转动惯量
GS.T = T;
GS.t = 0;
GS.dx = zeros(4,1);

% Graph Handle
figure(1); axis equal; hold on; view(0,0); set(gcf, 'position', [0,500,600,300]); axis([-1, 2, -.5, 1, -.5, 1]);
grid on;
% 绘制地面
ground_handle = patch([-2, 2, 2, -2], zeros(4,1), [0, 0, -0.5, -0.5], 'k', 'facealpha', 1, 'HandleVisibility', 'off');
% hatchfill(ground_handle, 'single', 45, 5, [1,1,1]);

[GS.pen.x, GS.pen.y, GS.pen.z]=cylinder(GS.pen.R, GS.pen.N);    % 创建以(0,0)为圆心，高度为[0,1]，半径为R的圆柱
GS.pen.z  = -GS.pen.len*GS.pen.z;                               % 高度放大len倍
GS.handle.pen = mesh(GS.pen.x+GS.pen.bias(1), GS.pen.y+GS.pen.bias(2), GS.pen.z+GS.pen.bias(3), ...
                  'FaceColor', GS.pen.color, 'FaceAlpha',0.5,...
                  'EdgeColor', 'k','EdgeAlpha',0,...
                  'LineStyle', '-','HandleVisibility','off');
GS.car.x  = ones(2,1)*[-GS.car.size(1), GS.car.size(1), GS.car.size(1), -GS.car.size(1), -GS.car.size(1)]/2;
GS.car.y  = ones(2,1)*[-GS.car.size(2), -GS.car.size(2), GS.car.size(2), GS.car.size(2), -GS.car.size(2)]/2+GS.car.size(2)/2;
GS.car.z  = [ones(1,5)*GS.car.size(3); -ones(1,5)*GS.car.size(3)]/2;

GS.car.x  = GS.car.size(1)*ones(2,1)*cos(linspace(0, 2*pi, 20));
GS.car.z  = GS.car.size(2)*ones(2,1)*sin(linspace(0, 2*pi, 20));
GS.car.y  = zeros(size(GS.car.x));
GS.handle.car(1) = mesh(GS.car.x+GS.car.bias(1), GS.car.y+GS.car.bias(2), GS.car.z+GS.car.bias(3), ...
                      'FaceColor', GS.car.color, 'FaceAlpha',0.5,...
                      'EdgeColor', 'k','EdgeAlpha',1,...
                      'LineStyle', '-','HandleVisibility','off');
GS.handle.car(2) = patch(GS.car.x(1,:)+GS.car.bias(1), GS.car.y(1,:)+GS.car.bias(2), GS.car.z(1,:)+GS.car.bias(3), ...
                         GS.car.color, 'FaceAlpha',0.5,...
                         'EdgeColor', 'k','EdgeAlpha',1,...
                         'LineStyle', '-','HandleVisibility','off');
GS.handle.car(3) = patch(GS.car.x(2,:)+GS.car.bias(1), GS.car.y(2,:)+GS.car.bias(2), GS.car.z(2,:)+GS.car.bias(3), ...
                         GS.car.color, 'FaceAlpha',0.5,...
                         'EdgeColor', 'k','EdgeAlpha',1,...
                         'LineStyle', '-','HandleVisibility','off');
                     

figure(2); hold on; set(gcf, 'position', [0,0,600,400]);
title('Pendulum');
GS.handle.theta  = animatedline('color', 'r', 'linewidth', 1, 'linestyle', '-');
GS.handle.thetav = animatedline('color', 'b', 'linewidth', 1, 'linestyle', '--');
GS.handle.thetaa = animatedline('color', 'k', 'linewidth', 1, 'linestyle', '-.');
% legend('\theta', '\theta v', '\theta a');

figure(3); hold on; set(gcf, 'position', [700,0,600,400]);
title('Car');
GS.handle.p = animatedline('color', 'r', 'linewidth', 1, 'linestyle', '-');
GS.handle.v = animatedline('color', 'b', 'linewidth', 1, 'linestyle', '--');
GS.handle.a = animatedline('color', 'k', 'linewidth', 1, 'linestyle', '-.');
% legend('p', 'v', 'a');

end

%% Update the graph
function GS = GraphUpdate(GS)
% 根据当前位置更新
GS.handle.car(1).XData    = GS.car.x+GS.car.bias(1);
GS.handle.car(1).YData    = GS.car.y+GS.car.bias(2);
GS.handle.car(1).ZData    = GS.car.z+GS.car.bias(3);
GS.handle.car(2).Vertices = [GS.car.x(1,:)+GS.car.bias(1); GS.car.y(1,:)+GS.car.bias(2); GS.car.z(1,:)+GS.car.bias(3)]';
GS.handle.car(3).Vertices = [GS.car.x(2,:)+GS.car.bias(1); GS.car.y(2,:)+GS.car.bias(2); GS.car.z(2,:)+GS.car.bias(3)]';

[GS.handle.pen.XData, GS.handle.pen.YData, GS.handle.pen.ZData] = ...
    rotatePendulum(GS.pen.x, GS.pen.y, GS.pen.z, GS.pen.theta, GS.car.bias);


addpoints(GS.handle.theta,  GS.t, GS.pen.theta);
addpoints(GS.handle.thetav, GS.t, GS.pen.thetav);
addpoints(GS.handle.thetaa, GS.t, GS.pen.thetaa);
addpoints(GS.handle.p, GS.t, GS.car.p);
addpoints(GS.handle.v, GS.t, GS.car.v);
addpoints(GS.handle.a, GS.t, GS.car.a);
% GS.pen.bias = GS.car.bias + [GS.pen.R; 0; GS.pen.R];
end

function [tmpX, tmpY, tmpZ] = rotatePendulum(x,y,z, theta, bias)
n = size(x, 2);
% R_y = [ cos(pi+theta), 0, sin(pi+theta);
%                  0, 1,          0;
%        -sin(pi+theta), 0, cos(pi+theta)];
R_y = [ cos(-theta), 0, sin(-theta);
                 0, 1,          0;
       -sin(-theta), 0, cos(-theta)];
for i = 1:n
    p1(:, i) = R_y*[x(1, i); y(1, i); z(1, i)];
    p2(:, i) = R_y*[x(2, i); y(2, i); z(2, i)];
end
tmpX = [p1(1, :); p2(1, :)]+bias(1);
tmpY = [p1(2, :); p2(2, :)]+bias(2);
tmpZ = [p1(3, :); p2(3, :)]+bias(3);
end

% System model
function GS = updateSystemModel(GS)
p = GS.car.bias(1);
v = GS.car.v;
a = GS.car.a;
u = GS.car.u;
theta  = GS.pen.theta;
thetav = GS.pen.thetav;
thetaa = GS.pen.thetaa;
L  = GS.pen.len;
mc = GS.car.mass;
mp = GS.pen.mass;
J  = GS.pen.J;
g  = GS.g;

cc = GS.car.c0;
cp = GS.pen.c0;

GS.dx = [v; ...
      thetav; ...
      inv([mc+mp, L*mp*cos(theta);  mp*L*cos(theta), J])*...
      [u-cc*v + mp*sin(theta)*thetav^2; ...
       -mp*g*L*sin(theta) - cp*thetav]];

x  = [p;theta;v;thetav] + GS.dx*GS.T;

if(GS.pen.theta - x(2) > pi)
    x(2) = x(2)-2*pi;
elseif(GS.pen.theta - x(2) < -pi)
    x(2) = x(2)+2*pi;
end
    

GS.car.bias(1)  = x(1); 
GS.car.p        = x(1);
GS.car.v        = x(3); 
GS.car.a        = GS.dx(1);
GS.pen.theta    = x(2);
GS.pen.thetav   = x(4);
GS.pen.thetaa   = GS.dx(2);
% fprintf('thetaa: %f\n', GS.pen.theta);
end