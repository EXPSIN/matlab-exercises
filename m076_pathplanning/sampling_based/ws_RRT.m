 Rapidly-exploring Random Tree
close all; clear; clc; 

figure(1);
hold on;
map = -double(rgb2gray(imread('newmap.png'))/255)';
% map = -double(rgb2gray(imread('env_03.bmp'))/255)';
resolution = 100;    % pix / m.
[Y, X] = meshgrid((1:size(map, 2))/resolution, (1:size(map, 1))/resolution);
mesh(X, Y, map);
colormap([1,1,1; 0,0,0]);

p_now    = [1; 1];
p_target = [2; 4];

plot([p_now(1), p_target(1)], [p_now(2), p_target(2)], '*', 'linestyle', 'none');

path = RRT_planning(map, p_now, p_target, resolution);

plot(path(1, :), path(2, :), 'b.-', 'markersize', 20);

function path = RRT_planning(map, p_now, p_target, resolution)
path.success = false;
path.res     = false;
path.map_size = size(map)' / resolution;

q = queue_add([], p_now, zeros(size(p_now)), 0);


while(true)
% x_rand = 0.1*p_target + rand(2, 1) .* path.map_size;
% x_rand = min(x_rand, path.map_size);
x_rand = rand(2, 1) .* path.map_size;
x_near = queue_near(q, x_rand);
x_new  = x_near + (x_rand - x_near) * min(0.5/norm(x_rand - x_near), 1);

if(is_safe(map, x_near, x_new, resolution))
    q = queue_add(q, x_new, x_near, norm(x_near-x_new));
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'r.-', 'markersize', 20); drawnow;
else
    continue;
end

if(norm(x_new-p_target) < 0.3)
    q = queue_add(q, p_target, x_new, norm(p_target-x_new));
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'r.-', 'markersize', 20); drawnow;
    break;
end

end

path = p_target; 
while(true)
    
    parent = queue_parent(q, path(:, 1));
    if(size(parent, 2) == 0)
        break;
    end
    path = [parent, path];
    
end
end

%{
    whether the edge safe
%}
function safe_flag = is_safe(map, x_near, x_new, resolution)
cnt = norm(x_near-x_new)*resolution;
% obstacle is -1
edge = [linspace(x_near(1)*resolution, x_new(1)*resolution, cnt); linspace(x_near(2)*resolution, x_new(2)*resolution, cnt)];
edge = max(round(edge), 1);
% plot(edge(1, :)/resolution, edge(2, :)/resolution, 'r-');
% map( sub2ind(size(map), edge(1, :), edge(2, :)) )
safe_flag = all(map( sub2ind(size(map), edge(1, :), edge(2, :)) ) == -1);
end


%{
    find parent
%}
function parent = queue_parent(q, key)
I = ismember(q.key', key', 'rows');
parent = q.parent(:, I);
end

%{
    add node to priority queue
%}
function q = queue_add(q, key, parent, dist)
if(isempty(q))
    q.key  = [];  % position
    q.parent = [];
    q.dist   = [];
end
q.key    = [key,    q.key];
q.parent = [parent, q.parent];
q.dist   = [dist,   q.dist];

% [q.dist, sort_index] = sort(q.dist);
% q.key    = q.key(:, sort_index);
% q.parent = q.parent(:, sort_index);
end

%{
    remove the nearset node in priority queue
%}
function q = queue_out(q, n)
q.key(:, 1:n)    = [];
q.parent(:, 1:n) = [];
q.dist(:, 1:n)   = [];
end

%{
    find safe neighbourhood
%}
function x_near = queue_near(q, x_rand)
[~, min_index] = min(vecnorm(x_rand  - q.key));
x_near         = q.key(:, min_index);
end
