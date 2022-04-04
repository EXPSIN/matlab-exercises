clf;
maxX = 2; 
maxY = 3; 
maxZ = 5; 
A = ones(maxX,maxY,maxZ);
A(2,3,4) = 0;
line1 = animatedline('Color', 'r', 'LineStyle','none', 'Marker', '.', 'MarkerSize', 100);
line2 = animatedline('Color', 'b', 'LineStyle','none', 'Marker', '.', 'MarkerSize', 100);
axis equal;
set(gcf, 'Color', [1,1,1]);
set(gca, 'FontSize', 15);
% alpha(line1, 0.5);
% alpha(line2, 0.5);
L = 0.9;
biasX = L*([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5);
biasY = L*([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5);
biasZ = L*([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5);
for i = 1:maxX
    for j = 1:maxY
        for k = 1:maxZ
            if(A(i,j,k) == 1)
                handle = patch(i+biasX, j+biasY, k+biasZ, 'c');
                alpha(handle, 0.8);
            else
                handle = patch(i+biasX, j+biasY, k+biasZ, 'k');
                alpha(handle, 0.5);
            end
        end
    end
end
xlabel('x');
ylabel('y');
zlabel('z');
            
for i = 1:360
    view(i, 29);
    buffer(i) = getframe(1);
    pause(0.01);
end

for idx = 1:4:size(buffer, 2)
    [A,map] = rgb2ind(frame2im(buffer(idx)), 256);
    if idx == 1
        imwrite(A,map,'buffer.gif','gif','LoopCount',Inf,'DelayTime',0.1);
    else
        imwrite(A,map,'buffer.gif','gif','WriteMode','append','DelayTime',0.1);
    end
end
