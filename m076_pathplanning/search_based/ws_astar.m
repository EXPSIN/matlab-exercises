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


i_now     = is2;
queue     = is2;
queue_val = norm(i_now - it2, 2);
searched_dist(i_now(1), i_now(2)) = 0;
searched_exp(i_now(1), i_now(2))  = 0;
searched_f(i_now(1), i_now(2))    = 0;
H = pp_graphic([], map, x_max, y_max);

while(true)
    % The first node
    i_now = queue(1, :);
    
    % Mark this point as searched.
    search_flag(i_now(1), i_now(2)) = 1.0;
    
    % Add the neibourhood to the queue
%     points = i_now + [1, 0; 0, 1; -1, 0; 0, -1];
    points   = i_now + [1, 0; 1, 1; 0, 1; -1, 1; -1, 0; -1, -1; 0, -1; 1, -1];
    dist_det = [1; sqrt(2); 1; sqrt(2); 1; sqrt(2); 1; sqrt(2)];
    
    % Filter 01: map range
    overlimit_flag = (points(:, 1) > x_max | points(:, 2) > y_max | points(:, 1) < x_min | points(:, 2) < y_min);
    dist_det(overlimit_flag, :) = [];
    points(overlimit_flag, :)   = [];
    
    for j = 1:size(points, 1)
        % Filter 02 03 04: searched, obstacle and whether existence in the euque
        if(search_flag(points(j, 1), points(j, 2)) == 1 || MAP(points(j, 1), points(j, 2)) == -1 || ismember(points(j,:),queue,'rows'))
            continue;
        end
        if(dist_det(j) == sqrt(2))
            det = points(j, :) - i_now;
            if((i_now(1) + det(1) <= x_max && MAP(i_now(1) + det(1), i_now(2) + 0) == -1) || (i_now(2) + det(2) <= y_max && MAP(i_now(1) + 0, i_now(2) + det(2)) == -1))
                continue;
            end
        end
        
        % update - parent node
        search_prev(points(j, 1), points(j, 2))   = sub2ind(size(MAP), i_now(1), i_now(2));

        % update - distance g(n)
        searched_dist(points(j, 1), points(j, 2)) = min(searched_dist(points(j, 1), points(j, 2)), searched_dist(i_now(1), i_now(2))+dist_det(j));

        % update - expect h(n)
        searched_exp(points(j, 1), points(j, 2)) = norm(points(j, :)-it2, 2);

        % update - f(n)
        searched_f(points(j, 1), points(j, 2)) = searched_dist(points(j, 1), points(j, 2)) + searched_exp(points(j, 1), points(j, 2));
        
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
    queue(1, :)     = [];
    queue_val(1, :) = [];
    
    
    drawnow;
    H.search_queue.XData = queue(:, 1)-0.5;
    H.search_queue.YData = queue(:, 2)-0.5;
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