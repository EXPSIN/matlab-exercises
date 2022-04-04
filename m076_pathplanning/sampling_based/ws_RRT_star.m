% Rapidly-exploring Random Tree
close all; clear; clc; 

figure(1);
hold on;
% map = -double(rgb2gray(imread('newmap.png'))/255)';
% map = -double(rgb2gray(imread('env_03.bmp'))/255)';
map = -double(rgb2gray(imread('env_01.png'))/255)';
resolution = 100;    % pix / m.
[Y, X] = meshgrid((1:size(map, 2))/resolution, (1:size(map, 1))/resolution);
mesh(X, Y, map);
colormap([1,1,1; 0,0,0]);
set(gcf, 'position', [0,0,size(map, 1),size(map, 2)]/resolution*100);

p_now    = [1; 1];
p_target = [7.5; 14];

plot([p_now(1), p_target(1)], [p_now(2), p_target(2)], '*', 'linestyle', 'none');

path = RRT_planning(map, p_now, p_target, resolution);

plot(path(1, :), path(2, :), 'b.-', 'markersize', 20);

function path = RRT_planning(map, p_start, p_target, resolution)
path.success = false;
path.res     = false;
path.map_size = size(map)' / resolution;

q = queue_add([], p_start, zeros(size(p_start)), 0);

find_count = 0 ;
while(true)
    find_count = find_count + 1;
    
    if(mod(find_count,2) == 1)
        x_rand = rand(2, 1) .* path.map_size;
    else
        x_rand = p_target;
    end
%     x_rand = rand(2, 1) .* path.map_size;
    
    x_near = queue_near(q, x_rand);
    x_new  = x_near + (x_rand - x_near) * min(0.1/norm(x_rand - x_near), 1);
    
    if(is_safe(map, x_near, x_new, resolution))
        near_flag  =  find_nearC(q, x_new, 1.0);
        parent_key =  choose_parent(q, near_flag, p_start, map, x_new, resolution);
        q = queue_add(q, x_new, parent_key, norm(parent_key-x_new));
        %     H(1) = plot(q.key(1, near_flag), q.key(2, near_flag), 'b.', 'linestyle', 'none', 'markersize', 20); drawnow;
        %     H(2) = plot([parent_key(1), x_new(1)], [parent_key(2), x_new(2)], 'c.--', 'markersize', 20); drawnow;
        %     delete(H);
        plot([parent_key(1), x_new(1)], [parent_key(2), x_new(2)], 'r.-', 'markersize', 10); drawnow limitrate;
    else
        continue;
    end
    
    if(norm(x_new-p_target) < 0.3)
        q = queue_add(q, p_target, x_new, norm(p_target-x_new));
        break;
    end
end

path = p_target;
while(true)
    
    parent = queue_parent(q, path(:, 1));
    if(isequal(parent, [0;0]))
        break;
    end
    path = [parent, path];
    
end
end

%% -----------------------

%{
    NearC
%}
function near_flag =  find_nearC(q, x_new, r_cir)
near_flag =  vecnorm(q.key - x_new) < r_cir;
end


%{
    NearC
%}
function parent_key = choose_parent(q, x_near_flag, p_start, map, x_new, resolution)
[~, parent_idx] = sort(vecnorm(q.key(:, x_near_flag) - p_start));
cnt = size(parent_idx, 2);
for i = 1:cnt
    I = find(x_near_flag, parent_idx(i));
    parent_key = q.key(:, I(end)); 
    if(is_safe(map, parent_key, x_new, resolution))
        break;
    end
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
