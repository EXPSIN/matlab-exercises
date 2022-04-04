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
i_now = is2;


% Initialize the search state
stop_flag = false;
search_list   = is2;
% search_prve   = zeros(1, 1);
search_prev   = zeros(x_max, y_max);
searched_flag = zeros(x_max, y_max);
searched_dist = inf(x_max, y_max);      % g(n)  
searched_exp  = inf(x_max, y_max);      % h(n)
searched_f    = inf(x_max, y_max);      % f(n)
searched_dist(i_now(1), i_now(2)) = 0;
searched_exp(i_now(1), i_now(2)) = 0;
searched_f(i_now(1), i_now(2)) = 0;


H = pp_graphic([], map, x_max, y_max);

while(true)
    % Generate search directions
    searched_flag(i_now(1), i_now(2)) = 1.0;
    
    % the points need to be extended
    search_tmp = search_list;
    
    % The search list
    clr = linspace(0.1, 0.9, size(search_tmp, 1))' * [0,0,1];
    
    for i = size(search_tmp, 1):-1:1
        searched_flag(search_tmp(i, 1), search_tmp(i, 2)) = 1.0;
        points = search_tmp(i, :) + [1, 0; 0, 1; -1, 0; 0, -1];
        points((points(:, 1) > x_max | points(:, 2) > y_max | points(:, 1) < x_min | points(:, 2) < y_min), :) = [];
        for j = size(points, 1):-1:1
            if(searched_flag(points(j, 1), points(j, 2)) == 1 || MAP(points(j, 1), points(j, 2)) == -1 || ismember(points(j,:),search_list,'rows'))
                points(j, :) = [];
                continue;
            end
            % update - parent node
            search_prev(points(j, 1), points(j, 2))   = sub2ind(size(MAP), search_tmp(i, 1), search_tmp(i, 2));

            % update - distance g(n)
            searched_dist(points(j, 1), points(j, 2)) = min(searched_dist(points(j, 1), points(j, 2)), searched_dist(search_tmp(i, 1), search_tmp(i, 2))+1);
            
            % update - expect h(n)
%             searched_exp(points(j, 1), points(j, 2)) = abs(points(j, 1)-it2(1)) + abs(points(j, 2)-it2(2));
            searched_exp(points(j, 1), points(j, 2)) = norm(points(j, :)-it2);
            
            % update - f(n)
            searched_f(points(j, 1), points(j, 2)) = min(searched_f(points(j, 1), points(j, 2)), searched_dist(points(j, 1), points(j, 2)) + searched_exp(points(j, 1), points(j, 2)));
            

            % update - text
            H.text(points(j, 1), points(j, 2)).String = sprintf('%d', int32(searched_dist(points(j, 1), points(j, 2))));
            if(isequal(points(j, :), it2))
                i_now = it2;
                stop_flag = true;
                break;
            end
        end
        % update
        search_list = mat_replace(search_list, i, points);
    end
    
    fprintf('%d\n', size(search_list, 1));
    H.search_list.XData = search_list(:, 1)-0.5;
    H.search_list.YData = search_list(:, 2)-0.5;
    search_n = size(search_list, 1);
    drawnow;
    % Stop condition
    if(stop_flag || search_n == 0)
        break;
    end
    
    list_index = sub2ind(size(MAP), search_list(:, 1), search_list(:, 2));
    [~, sort_index] = sort(searched_f(list_index)); % small -> big
    list_index = list_index(sort_index);
    
    i_now = search_list(sort_index(1), :);
     
end



% Obtain Path.
if(stop_flag && isequal(i_now, it2))
    path = it2;
    while(~isequal(i_now, is2))
        path  = [i_now; path];
        [i_now(1, 1), i_now(1, 2)] = ind2sub([x_max, y_max], search_prev(i_now(1), i_now(2)));
    end
    path = [is2; path];
else
    fprintf('There is no path to the target point.');
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
H.search_list = scatter(-1.5, -1.5, 210, 'b', 'filled');
H.start_target_point = scatter(map([1,end], 1)-0.5,map([1,end], 2)-0.5, 200, 'k','filled');
H.path = plot(-1, -1, 'c-', 'linewidth', 5);
scatter(map(2:end-1, 1)-0.5,map(2:end-1, 2)-0.5, 200, 'r','filled');

for i = 1:x_max
    for j = 1:y_max
        H.text(i, j) = text(i-0.5, j-0.5, '', 'HorizontalAlignment', 'center');
    end
end

end




function mat = mat_replace(mat, line, mat_new)
mat = [mat(1:(line-1), :); mat_new; mat((line+1):end, :)];
end