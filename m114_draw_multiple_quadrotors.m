% expsin

clear; close all; clc;

x = [0, 2, 3];
y = [0,-2, 0];
z = [0, 0, 1];
roll  = [pi/4, 0, 0];
pitch = [0, pi/6, 0];
yaw   = [0, 0, pi/3];
color = 'rbk';
len = 0.5;
radius = 0.2;
count = length(x);

% 1. 静态显示
figure(1); 
axis equal; view(3); grid on;
handle1 = drawMultiQuad(x, y, z, roll, pitch, yaw, color, len, radius, count);
% 如果需要删除的话：
% delete(handle1)


% 2. 实时更新
figure(2); 
axis equal; view(3); grid on;
for i = 1:100
    
    if(exist('handle2', 'var'))
        delete(handle2);
    end
    
    x = [cos(i/100*4*pi), 2, 3];
    y = [sin(i/100*4*pi),-2, 0];
    z = [sin(i/100*2*pi), 0, 1];
    roll  = [0, 0, 0];
    pitch = [0, 0, 0];
    yaw   = [0, 0, 0];
    handle2 = drawMultiQuad(x, y, z, roll, pitch, yaw, color, len, radius, count);

    pause(0.05);
end





%{
x       x轴位置（m），向量，大小为N
y       y轴位置（m），向量，大小为N
z       z轴位置（m），向量，大小为N
roll    roll轴角度（rad），向量，大小为N
pitch   pitch轴角度（rad），向量，大小为N
yaw     yaw轴角度（rad），向量，大小为N
CLR     颜色，向量，大小为N
L       对角电机距离（m）
R       桨叶半径（m）
N       飞行器数量
%}
function Handle = drawMultiQuad(x, y, z, roll, pitch, yaw, CLR, L, R, N)
% drawMultiQuad(0, 0, 0, 0, 0, 0, 2, 0.5, 'r')
% persistent Handle keyPoint planeCNT motor cntPoints cntCP QuadPoints;

keyPoint = [L/2, L/2, 0; -L/2, L/2, 0;  -L/2, -L/2, 0;  L/2, -L/2, 0];
planeCNT = N;
cirPoints = [R*sin(linspace(0, 2*pi, 20))', R*cos(linspace(0, 2*pi, 20))'];
cntCP = size(cirPoints, 1);   %cntCP
cirPoints(:,3) = 0;
for mIdx = 1:4
    motor(cntCP*(mIdx-1)+1:cntCP*mIdx,:) = keyPoint(mIdx,:) + cirPoints;
end
QuadPoints = [keyPoint; motor];
cntPoints = size(QuadPoints, 1);

Frame = zeros(cntPoints, 3, planeCNT);

for i = 1:planeCNT
    if(roll(1) ~= 0 || pitch(1) ~= 0 || yaw(1) ~= 0)
        quaternion = angle2quat(roll(i), pitch(i), yaw(i), 'XYZ');
        for idx = 1:cntPoints
            Frame(idx, :, i) = [x(i), y(i), z(i)] + quatrotate(quaternion, QuadPoints(idx, :));
        end
    else
        Frame(:, :, i) = [x(i), y(i), z(i)] + QuadPoints(:, :);
    end
end
picHandle = [];
for i = 1:planeCNT
    temp = line(Frame([1,3],1,i), Frame([1,3],2,i), Frame([1,3],3,i),'Color',CLR(i), 'LineWidth', 3); picHandle = [picHandle temp];
    temp = line(Frame([2,4],1,i), Frame([2,4],2,i), Frame([2,4],3,i),'Color',CLR(i), 'LineWidth', 3); picHandle = [picHandle temp];
    temp = line(Frame(5        :4+  cntCP,1,i), Frame(5        :4+  cntCP,2,i), Frame(5        :4+  cntCP,3,i),'Color',CLR(i), 'LineWidth', 1); picHandle = [picHandle temp];
    temp = line(Frame(5+  cntCP:4+2*cntCP,1,i), Frame(5+  cntCP:4+2*cntCP,2,i), Frame(5+  cntCP:4+2*cntCP,3,i),'Color',CLR(i), 'LineWidth', 1); picHandle = [picHandle temp];
    temp = line(Frame(5+2*cntCP:4+3*cntCP,1,i), Frame(5+2*cntCP:4+3*cntCP,2,i), Frame(5+2*cntCP:4+3*cntCP,3,i),'Color',CLR(i), 'LineWidth', 1); picHandle = [picHandle temp];
    temp = line(Frame(5+3*cntCP:4+4*cntCP,1,i), Frame(5+3*cntCP:4+4*cntCP,2,i), Frame(5+3*cntCP:4+4*cntCP,3,i),'Color',CLR(i), 'LineWidth', 1); picHandle = [picHandle temp];
end
Handle = picHandle;
end