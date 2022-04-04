%{
    @arg map
    Obstacle=-1, Target = 0, Start=1
%}
function path = ws_astar(map)

% The upper bound and the lower bound of map.
x_max = max(map(:, 1));
y_max = max(map(:, 2));
x_min = 1;
y_min = 1;

% generate map
MAP   = 2*(ones(x_max, y_max));
MAP(sub2ind(size(MAP), map(2:end-1, 1), map(2:end-1, 2))) = -1;     % Obstacle
MAP(map(1,   1),     map(1,   2))     = 1;      % start
MAP(map(end, 1),     map(end, 2))     = 0;      % target

% Find obstacle, start point and target.
io = find(MAP == -1); [io2(:, 1), io2(:, 2)] = ind2sub([x_max, y_max], io);
it = find(MAP == 0);  [it2(:, 1), it2(:, 2)] = ind2sub([x_max, y_max], it);
is = find(MAP == 1);  [is2(:, 1), is2(:, 2)] = ind2sub([x_max, y_max], is);



% Initialize the search state
arrival_flag = false;
search_prev = zeros(x_max, y_max);
search_flag = zeros(x_max, y_max);
searched_dist = inf(x_max, y_max);      % g(n)  
searched_exp  = inf(x_max, y_max);      % h(n)
searched_f    = inf(x_max, y_max);      % f(n)
L_map         = map2len(MAP, x_max, y_max);


i_now     = is2;
queue     = is2;
queue_val = norm(i_now - it2, 2);
searched_dist(i_now(1), i_now(2)) = 0;
searched_exp(i_now(1), i_now(2))  = 0;
searched_f(i_now(1), i_now(2))    = 0;

H = pp_graphic([], map, x_max, y_max);
% search_times = 0;

while(true)
%     search_times = search_times + 1;
    % The first node
    i_now = queue(1, :);
    
    % Mark this point as searched.
    search_flag(i_now(1), i_now(2)) = 1.0;
    
    % Get the neighborhood to the queue
    points = i_now + [1, 1; -1, 1; -1, -1; 1, -1];  % 最多产生4个neighborhood
%     dirs  = [1, 1; -1, 1; -1, -1; 1, -1];
%     dirs2 = [1, 0;  0, 1; -1, 0; 0, -1];
    
    dirs = [1, 1; -1, 1; -1, -1; 1, -1; 1, 0;  0, 1; -1, 0; 0, -1];
    points = i_now + dirs;
    
    valued_dirs = true(size(dirs,1), 1);
    
    if((points(5, 1) >= 1 && points(5, 1) <= x_max && points(5, 2) >= 1 && points(5, 2) <= y_max) && MAP(points(5, 1), points(5, 2)) == -1)
        valued_dirs(1) = false;
        valued_dirs(4) = false;
        valued_dirs(5) = false;
    end
    if((points(6, 1) >= 1 && points(6, 1) <= x_max && points(6, 2) >= 1 && points(6, 2) <= y_max) && MAP(points(6, 1), points(6, 2)) == -1)
        valued_dirs(1) = false;
        valued_dirs(2) = false;
        valued_dirs(6) = false;
    end
    if((points(7, 1) >= 1 && points(7, 1) <= x_max && points(7, 2) >= 1 && points(7, 2) <= y_max) && MAP(points(7, 1), points(7, 2)) == -1)
        valued_dirs(2) = false;
        valued_dirs(3) = false;
        valued_dirs(7) = false;
    end
    if((points(8, 1) >= 1 && points(8, 1) <= x_max && points(8, 2) >= 1 && points(8, 2) <= y_max) && MAP(points(8, 1), points(8, 2)) == -1)
        valued_dirs(3) = false;
        valued_dirs(4) = false;
        valued_dirs(8) = false;
    end
    
    
    dirs = dirs(valued_dirs, :);
    points = points(valued_dirs, :);
%     points = i_now + dirs;
    
    for i = 1:size(dirs, 1)

        % Follow the direction of dir(i, :), the length is
        L0 = [0, 0];
        if(dirs(i, 1) == 1)
            L0(1) = L_map(1, i_now(1), i_now(2));
        else
            L0(1) = L_map(3, i_now(1), i_now(2));
        end
        if(dirs(i, 2) == 1)
            L0(2) = L_map(2, i_now(1), i_now(2));
        else
            L0(2) = L_map(4, i_now(1), i_now(2));
        end
        
        L  = 0;
        while(true)
            L = L + 1;
            points(i, :) = i_now + dirs(i, :) * L;
            
            if((points(i, 1) > x_max || points(i, 2) > y_max || points(i, 1) < x_min || points(i, 2) < y_min))
                break;
            end
            
            L_now = [0,0];
            if(dirs(i, 1) == 1)
                L_now(1) = L_map(1, points(i, 1), points(i, 2));
            else
                L_now(1) = L_map(3, points(i, 1), points(i, 2));
            end
            if(dirs(i, 2) == 1)
                L_now(2) = L_map(2, points(i, 1), points(i, 2));
            else
                L_now(2) = L_map(4, points(i, 1), points(i, 2));
            end
            
            
            
            if(~isequal(L0, L_now))
                break;
            end
        end
    end
    
    
    
    
    % Filter 01: map range
    points((points(:, 1) > x_max | points(:, 2) > y_max | points(:, 1) < x_min | points(:, 2) < y_min), :) = [];
    for j = size(points, 1):-1:1
        % Filter 02 03 04: searched, obstacle and whether existence in the euque
        if(search_flag(points(j, 1), points(j, 2)) == 1 || MAP(points(j, 1), points(j, 2)) == -1 || ismember(points(j,:),queue,'rows'))
            points(j, :) = [];
            continue;
        end
        
        % update - parent node
        search_prev(points(j, 1), points(j, 2))   = sub2ind(size(MAP), i_now(1), i_now(2));

        % update - distance g(n)
        searched_dist(points(j, 1), points(j, 2)) = min(searched_dist(points(j, 1), points(j, 2)), searched_dist(i_now(1), i_now(2)) + norm(i_now-points(j, :)) );
        
        
        % update - expect h(n)
        searched_exp(points(j, 1), points(j, 2)) = norm(points(j, :)-it2, 2);

        % update - f(n)
        searched_f(points(j, 1), points(j, 2)) = min(searched_f(points(j, 1), points(j, 2)), searched_dist(points(j, 1), points(j, 2)) + searched_exp(points(j, 1), points(j, 2)));
        
        % Add to queue
        queue     = [queue; points(j, :)];
        queue_val = [queue_val; searched_f(points(j, 1), points(j, 2))];
        
        % update - text
        H.text(points(j, 1), points(j, 2)).String = sprintf('%d', int32(searched_dist(points(j, 1), points(j, 2))));
    end
    
    % Stop condition
    arrival_flag = ismember(it2, queue, 'rows');
    if(arrival_flag || size(queue, 1) == 1)
        break;
    end
    
    % out queue
    [queue_val, sort_index] = sort(queue_val);
    queue = queue(sort_index, :);
    
    H.search_queue.XData = queue(:, 1)-0.5;
    H.search_queue.YData = queue(:, 2)-0.5;
    [in_falg, now_index] = ismember(i_now, queue, 'rows');
    if(in_falg)
        queue_val(now_index, :) = [];
        queue(now_index, :) = [];
    else
        queue(1, :)     = [];
        queue_val(1, :) = [];
    end
   
    
    
    H.search_queue.XData = queue(:, 1)-0.5;
    H.search_queue.YData = queue(:, 2)-0.5;
    drawnow;
end



% Obtain Path.
if(arrival_flag)
    path = it2;
    i_now = it2;
    while(~isequal(i_now, is2))
        path  = [i_now; path];
        [i_now(1, 1), i_now(1, 2)] = ind2sub([x_max, y_max], search_prev(i_now(1), i_now(2)));
    end
    path = [is2; path];
else
    fprintf('There is no path to the target point.\n');
    path = zeros(0, 2);
end


H.path.XData = path(:, 1)-0.5;
H.path.YData = path(:, 2)-0.5;

end

function H = pp_graphic(H, map, x_max, y_max)
figure;
hold on;
grid on;
axis equal;
set(gca,'xtick',0:1:x_max);
set(gca,'ytick',0:1:y_max);
axis([0, x_max, 0, y_max]);
set(gcf, 'position', [0,0,800,500], 'color', 'w')
H.search_queue = scatter(-1.5, -1.5, 210, 'c', 'filled');
H.start_target_point = scatter(map([1,end], 1)-0.5,map([1,end], 2)-0.5, 200, 'k','filled');
H.path = plot(-1, -1, 'c-', 'linewidth', 5);
scatter(map(2:end-1, 1)-0.5,map(2:end-1, 2)-0.5, 200, 'r','filled');

H.search_now = scatter(-1.5, -1.5, 180, 'c', 'filled');

for i = 1:x_max
    for j = 1:y_max
        H.text(i, j) = text(i-0.5, j-0.5, '', 'HorizontalAlignment', 'center');
    end
end

end




function mat = mat_replace(mat, line, mat_new)
mat = [mat(1:(line-1), :); mat_new; mat((line+1):end, :)];
end




function L = map2len(MAP, x_max, y_max)
L = zeros(4, x_max, y_max);

dir = [1, 0; 0, 1; -1, 0; 0, -1];
for i = 1:x_max
    for j = 1:y_max
        L(1, i, j) = get_length(MAP, [i, j], dir(1, :), x_max, y_max);
        L(2, i, j) = get_length(MAP, [i, j], dir(2, :), x_max, y_max);
        L(3, i, j) = get_length(MAP, [i, j], dir(3, :), x_max, y_max);
        L(4, i, j) = get_length(MAP, [i, j], dir(4, :), x_max, y_max);
    end
end

end

function L = get_length(MAP, i_now, dir, x_max, y_max)
L = 0;
while(true)
    L = L + 1;
    i_s = i_now + dir*L;
    if( i_s(1) < 1 || i_s(1) > x_max || i_s(2) < 1 || i_s(2) > y_max)
        L = L - 1;
        break;
    elseif(MAP(i_s(1), i_s(2)) == -1)
        L = L - 1;
        break;
    end
end
end