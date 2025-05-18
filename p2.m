% 无人机校园配送路径规划系统（含A*曲线避障）
clear all;
close all;
clc;

% ====================== 数据准备 ======================
% 示例数据：实际使用时替换为您的坐标数据
% 格式: [站点编号, 经度, 纬度]
% 0号为配送服务站，1-25号为目的地
delivery_data = [
    0, 120.719393, 30.736683;   % 0号点
    1, 120.723462, 30.738040;    % 1号点
    2, 120.723064, 30.737176;    % 2号点
    3, 120.724139, 30.738074;    % 3号点
    4, 120.720783, 30.736567;    % 4号点
    5, 120.719332, 30.738759;    % 5号点
    6, 120.717844, 30.736583;    % 6号点
    7, 120.723751, 30.737023;    % 7号点
    8, 120.723722, 30.737300;    % 8号点
    9, 120.721873, 30.738392;    % 9号点
    10, 120.723267, 30.738555;   % 10号点
    11, 120.715796, 30.739321;   % 11号点
    12, 120.716573, 30.739419;   % 12号点
    13, 120.716315, 30.738643;   % 13号点
    14, 120.713438, 30.739864;   % 14号点
    15, 120.712416, 30.739130;   % 15号点
    16, 120.710881, 30.737107;   % 16号点
    17, 120.708420, 30.738185;   % 17号点
    18, 120.707961, 30.737138;   % 18号点
    19, 120.707201, 30.738079;   % 19号点
    20, 120.709221, 30.737340;   % 20号点
    21, 120.707649, 30.736115;   % 21号点
    22, 120.709162, 30.736004;   % 22号点
    23, 120.709125, 30.736497;   % 23号点
    24, 120.709164, 30.736937;   % 24号点
    25, 120.710817, 30.738949    % 25号点
];

% 禁飞区数据（保持不变）
no_fly_zones = [
    1, 120.72393, 30.740433, 50;    % 1号楼
    2, 120.723586, 30.741172, 50;    % 2号楼
    3, 120.722391, 30.74125, 50;    % 3号楼
    4, 120.723014, 30.740492, 50;    % 4号楼
    5, 120.721197, 30.740929, 50;    % 5号楼
    6, 120.72089, 30.737473, 50;    % 6号楼
    7, 120.720759, 30.738062, 50;    % 7号楼
    8, 120.720589, 30.738516, 50;    % 8号楼
    9, 120.719913, 30.737797, 50;    % 9号楼
    10, 120.719985, 30.739815, 50;    % 10号楼
    11, 120.719838, 30.740297, 50;    % 11号楼
    12, 120.719368, 30.740938, 50;    % 12号楼
    13, 120.718932, 30.740054, 50;    % 13号楼
    14, 120.719011, 30.739693, 50;    % 14号楼
    15, 120.715882, 30.73669, 50;    % 15号楼
    16, 120.714339, 30.73641, 50;    % 16号楼
    17, 120.712602, 30.736237, 50;    % 17号楼
    18, 120.711484, 30.735812, 50;    % 18号楼
    19, 120.711426, 30.736387, 50;    % 19号楼
    20, 120.711147, 30.738363, 50;    % 20号楼
    21, 120.716283, 30.736263, 50;    % 张子良教学楼
    22, 120.714374, 30.739401, 50;    % 行政楼
    23, 120.713199, 30.738302, 50;    % 图书馆
];

% ====================== 参数设置 ======================
earth_radius = 6371000; % 地球半径(米)
grid_size = 10;         % A*算法网格大小（米）

% 蚁群算法参数（可根据需要调整）
num_ants = 50; max_iterations = 200; alpha = 1; beta = 5; rho = 0.5; Q = 100;
num_drones = 3;          % 无人机数量

% ====================== 数据预处理 ======================
num_sites = size(delivery_data, 1);
latitudes = delivery_data(:, 3); longitudes = delivery_data(:, 2);

% 经纬度转平面坐标
center_lat = mean(latitudes); center_lon = mean(longitudes);
x = (longitudes - center_lon) * pi/180 * earth_radius * cos(center_lat * pi/180);
y = (latitudes - center_lat) * pi/180 * earth_radius;

% 禁飞区坐标转换
num_no_fly = size(no_fly_zones, 1);
no_fly_x = (no_fly_zones(:,2) - center_lon) * pi/180 * earth_radius * cos(center_lat * pi/180);
no_fly_y = (no_fly_zones(:,3) - center_lat) * pi/180 * earth_radius;
no_fly_r = no_fly_zones(:,4);

% ====================== 网格地图构建 ======================
x_min = min(x); x_max = max(x); y_min = min(y); y_max = max(y);
grid_x = ceil((x_max - x_min)/grid_size); grid_y = ceil((y_max - y_min)/grid_size);
grid_map = false(grid_x, grid_y); % 障碍物地图（true为障碍物）

% 标记禁飞区对应的网格
for i = 1:num_no_fly
    cx = no_fly_x(i); cy = no_fly_y(i); r = no_fly_r(i);
    x_start = max(floor((cx - r - x_min)/grid_size)+1, 1);
    x_end = min(ceil((cx + r - x_min)/grid_size), grid_x);
    y_start = max(floor((cy - r - y_min)/grid_size)+1, 1);
    y_end = min(ceil((cy + r - y_min)/grid_size), grid_y);
    
    for xi = x_start:x_end
        for yi = y_start:y_end
            grid_cx = x_min + (xi-0.5)*grid_size;
            grid_cy = y_min + (yi-0.5)*grid_size;
            if sqrt((grid_cx-cx)^2 + (grid_cy-cy)^2) <= r
                grid_map(xi, yi) = true;
            end
        end
    end
end

% 无人机校园配送路径规划系统（含A*曲线避障）
% （保持其他部分不变...）

% ====================== A*路径规划函数 ======================
function [dist, path] = astar_avoid(start_x, start_y, end_x, end_y, grid_map, grid_size, x_min, y_min)
    % 坐标转网格索引
    sx = floor((start_x - x_min)/grid_size) + 1;
    sy = floor((start_y - y_min)/grid_size) + 1;
    ex = floor((end_x - x_min)/grid_size) + 1;
    ey = floor((end_y - y_min)/grid_size) + 1;
    
    % 边界检查
    if sx < 1 || sx > size(grid_map,1) || sy < 1 || sy > size(grid_map,2)
        dist = inf; path = []; return;
    end
    if ex < 1 || ex > size(grid_map,1) || ey < 1 || ey > size(grid_map,2)
        dist = inf; path = []; return;
    end
    if grid_map(sx,sy) || grid_map(ex,ey)
        dist = inf; path = []; return;
    end
    
    % 8邻域偏移
    dirs = [-1 -1; 0 -1; 1 -1;
            -1  0;       1  0;
            -1  1; 0  1; 1  1];
    dir_cost = [sqrt(2); 1; sqrt(2); 1; 1; sqrt(2); 1; sqrt(2)];
    
    % 初始化A*
    open_list = zeros(size(grid_map)); % 开放列表索引
    closed_list = false(size(grid_map)); % 关闭列表
    g_score = inf(size(grid_map)); % G值
    h_score = zeros(size(grid_map)); % H值
    f_score = inf(size(grid_map)); % F值
    came_from = zeros(size(grid_map)); % 父节点索引
    
    % 设置起点
    g_score(sx,sy) = 0;
    h_score(sx,sy) = grid_size*(abs(sx-ex) + abs(sy-ey));
    f_score(sx,sy) = h_score(sx,sy);
    open_list(sx,sy) = 1; % 1表示在开放列表中
    
    % 开放列表优先队列（使用F值排序）
    open_queue = [sx, sy, f_score(sx,sy)];
    
    while ~isempty(open_queue)
        % 选择F值最小的节点
        [~, idx] = min(open_queue(:,3));
        current = open_queue(idx,:);
        open_queue(idx,:) = [];
        cx = current(1); cy = current(2);
        
        % 标记为已访问
        open_list(cx,cy) = 0;
        closed_list(cx,cy) = true;
        
        % 到达终点
        if cx == ex && cy == ey
            path = reconstruct_path(came_from, cx, cy, grid_size, x_min, y_min);
            dist = g_score(cx,cy);
            return;
        end
        
        % 扩展邻域节点
        for d = 1:size(dirs,1)
            nx = cx + dirs(d,1);
            ny = cy + dirs(d,2);
            
            % 检查边界和障碍物
            if nx < 1 || nx > size(grid_map,1) || ny < 1 || ny > size(grid_map,2)
                continue;
            end
            if closed_list(nx,ny) || grid_map(nx,ny)
                continue;
            end
            
            % 计算G值
            tentative_g = g_score(cx,cy) + grid_size * dir_cost(d);
            
            % 如果是新节点或找到更优路径
            if open_list(nx,ny) == 0 || tentative_g < g_score(nx,ny)
                % 记录路径
                came_from(nx,ny) = sub2ind(size(grid_map), cx, cy);
                
                % 更新评分
                g_score(nx,ny) = tentative_g;
                h_score(nx,ny) = grid_size*(abs(nx-ex) + abs(ny-ey));
                f_score(nx,ny) = tentative_g + h_score(nx,ny);
                
                % 添加到开放列表
                if open_list(nx,ny) == 0
                    open_list(nx,ny) = 1;
                    open_queue = [open_queue; nx, ny, f_score(nx,ny)];
                else
                    % 更新优先队列中的F值
                    for i = 1:size(open_queue,1)
                        if open_queue(i,1) == nx && open_queue(i,2) == ny
                            open_queue(i,3) = f_score(nx,ny);
                            break;
                        end
                    end
                end
            end
        end
    end
    
    % 路径重构函数
    function path = reconstruct_path(came_from, cx, cy, grid_size, x_min, y_min)
        path = [];
        while came_from(cx,cy) ~= 0
            path = [path; cx, cy];
            [px, py] = ind2sub(size(came_from), came_from(cx,cy));
            cx = px; cy = py;
        end
        path = [path; cx, cy]; % 添加起点
        path = flipud(path); % 反转顺序
        
        % 转换为实际坐标
        path = [x_min + (path(:,1)-0.5)*grid_size, y_min + (path(:,2)-0.5)*grid_size];
    end
end

% （保持其他部分不变...）

% ====================== 距离矩阵计算（含A*避障） ======================
distance_matrix = zeros(num_sites, num_sites);
path_storage = cell(num_sites, num_sites); % 存储路径点

for i = 1:num_sites
    for j = i+1:num_sites
        [dist, path] = astar_avoid(x(i), y(i), x(j), y(j), grid_map, grid_size, x_min, y_min);
        if isempty(path) || isinf(dist)
            % 无法避障时使用原始距离（直线可能穿过禁飞区）
            dist = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
            path = [x(i), y(i); x(j), y(j)];
        end
        distance_matrix(i,j) = dist;
        distance_matrix(j,i) = dist;
        path_storage{i,j} = path;
        path_storage{j,i} = flipud(path);
    end
end

% ====================== 单无人机路径规划（蚁群算法） ======================
% （保持蚁群算法逻辑不变，仅使用新的distance_matrix）
pheromone_matrix = ones(num_sites, num_sites);
pheromone_matrix(logical(eye(num_sites))) = 0;

best_path = []; best_dist = inf;
distance_history = zeros(max_iterations, 1);

for iter = 1:max_iterations
    % 蚂蚁路径构建
    all_paths = zeros(num_ants, num_sites);
    all_dists = zeros(num_ants, 1);
    
    for ant = 1:num_ants
        unvisited = 2:num_sites;
        curr = 1; path = curr;
        
        for k = 2:num_sites
            if isempty(unvisited)
                break;
            end
            % 计算选择概率
            probs = pheromone_matrix(curr, unvisited).^alpha .* (1./distance_matrix(curr, unvisited)).^beta;
            probs = probs / sum(probs);
            next_node = unvisited(randsample(length(unvisited), 1, true, probs));
            
            path = [path, next_node];
            unvisited(unvisited==next_node) = [];
            curr = next_node;
        end
        
        % 计算总距离
        total_dist = 0;
        for m = 1:num_sites-1
            total_dist = total_dist + distance_matrix(path(m), path(m+1));
        end
        total_dist = total_dist + distance_matrix(path(end), 1);
        
        all_paths(ant,:) = path;
        all_dists(ant) = total_dist;
        
        % 更新最优解
        if total_dist < best_dist
            best_dist = total_dist;
            best_path = path;
        end
    end
    
    % 信息素更新
    pheromone_matrix = (1-rho)*pheromone_matrix;
    for ant = 1:num_ants
        path = all_paths(ant,:);
        for m = 1:num_sites-1
            pheromone_matrix(path(m), path(m+1)) = pheromone_matrix(path(m), path(m+1)) + Q/all_dists(ant);
            pheromone_matrix(path(m+1), path(m)) = pheromone_matrix(path(m), path(m+1));
        end
        pheromone_matrix(path(end), 1) = pheromone_matrix(path(end), 1) + Q/all_dists(ant);
        pheromone_matrix(1, path(end)) = pheromone_matrix(path(end), 1);
    end
    
    distance_history(iter) = best_dist;
end

% ====================== 可视化单无人机路径 ======================
figure('Position', [100, 100, 1000, 800]);
hold on;

% 绘制禁飞区
for i = 1:num_no_fly
    theta = linspace(0,2*pi,100);
    plot(no_fly_x(i)+no_fly_r(i)*cos(theta), no_fly_y(i)+no_fly_r(i)*sin(theta), 'r-', 'LineWidth', 1);
    fill(no_fly_x(i)+no_fly_r(i)*cos(theta), no_fly_y(i)+no_fly_r(i)*sin(theta), 'r', 'FaceAlpha', 0.2);
end

% 绘制站点
plot(x(1), y(1), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(x(1)+10, y(1)+10, '配送站', 'FontSize', 10);
plot(x(2:end), y(2:end), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
for i = 2:num_sites
    text(x(i)+10, y(i)+10, num2str(i-1), 'FontSize', 10);
end

% 绘制A*避障路径
complete_path = [best_path, 1];
for i = 1:length(complete_path)-1
    curr_node = complete_path(i);
    next_node = complete_path(i+1);
    path = path_storage{curr_node, next_node};
    plot(path(:,1), path(:,2), 'g-', 'LineWidth', 2);
end

title('单无人机配送路径规划结果（A*避障）', 'FontSize', 14);
xlabel('X坐标 (米)', 'FontSize', 12);
ylabel('Y坐标 (米)', 'FontSize', 12);
grid on;
legend('禁飞区', '配送站', '目的地', '避障路径');
axis equal;

% ====================== 多无人机路径规划（K-means+蚁群） ======================
% （保持聚类逻辑不变，使用新的distance_matrix和path_storage）
coordinates = [x(2:end), y(2:end)];
[idx, ~] = kmeans(coordinates, num_drones, 'Distance', 'sqeuclidean', 'Replicates', 5);

drone_paths = cell(num_drones, 1);
drone_dists = zeros(num_drones, 1);

for drone = 1:num_drones
    dests = find(idx==drone) + 1;
    if isempty(dests)
        continue;
    end
    
    sub_sites = [1; dests];
    sub_num = length(sub_sites);
    sub_dist = distance_matrix(sub_sites, sub_sites');
    
    % 子问题蚁群算法
    sub_pheromone = ones(sub_num, sub_num);
    sub_pheromone(logical(eye(sub_num))) = 0;
    sub_best = inf;
    
    for s_iter = 1:max_iterations
        sub_all_paths = zeros(num_ants, sub_num);
        sub_all_dists = zeros(num_ants, 1);
        
        for s_ant = 1:num_ants
            sub_unvisited = 2:sub_num;
            sub_curr = 1; sub_path = sub_curr;
            
            for s_k = 2:sub_num
                if isempty(sub_unvisited)
                    break;
                end
                probs = sub_pheromone(sub_curr, sub_unvisited).^alpha .* (1./sub_dist(sub_curr, sub_unvisited)).^beta;
                probs = probs / sum(probs);
                sub_next = sub_unvisited(randsample(length(sub_unvisited), 1, true, probs));
                
                sub_path = [sub_path, sub_next];
                sub_unvisited(sub_unvisited==sub_next) = [];
                sub_curr = sub_next;
            end
            
            sub_total = 0;
            for s_m = 1:sub_num-1
                sub_total = sub_total + sub_dist(sub_path(s_m), sub_path(s_m+1));
            end
            sub_total = sub_total + sub_dist(sub_path(end), 1);
            
            sub_all_paths(s_ant,:) = sub_path;
            sub_all_dists(s_ant) = sub_total;
            
            if sub_total < sub_best
                sub_best = sub_total;
                sub_best_path = sub_path;
            end
        end
        
        % 子问题信息素更新
        sub_pheromone = (1-rho)*sub_pheromone;
        for s_ant = 1:num_ants
            s_path = sub_all_paths(s_ant,:);
            for s_m = 1:sub_num-1
                sub_pheromone(s_path(s_m), s_path(s_m+1)) = sub_pheromone(s_path(s_m), s_path(s_m+1)) + Q/sub_all_dists(s_ant);
            end
            sub_pheromone(s_path(end), 1) = sub_pheromone(s_path(end), 1) + Q/sub_all_dists(s_ant);
        end
    end
    
    real_path = sub_sites(sub_best_path);
    real_path = [real_path; 1];
    drone_paths{drone} = real_path;
    drone_dists(drone) = sub_best;
end

% ====================== 可视化多无人机路径 ======================
figure('Position', [100, 100, 1000, 800]);
hold on;
colors = lines(num_drones);

% 绘制禁飞区和站点（同上）
for i = 1:num_no_fly
    theta = linspace(0,2*pi,100);
    plot(no_fly_x(i)+no_fly_r(i)*cos(theta), no_fly_y(i)+no_fly_r(i)*sin(theta), 'r-', 'LineWidth', 1);
    fill(no_fly_x(i)+no_fly_r(i)*cos(theta), no_fly_y(i)+no_fly_r(i)*sin(theta), 'r', 'FaceAlpha', 0.2);
end
plot(x(1), y(1), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(x(1)+10, y(1)+10, '配送站', 'FontSize', 10);
plot(x(2:end), y(2:end), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
for i = 2:num_sites
    text(x(i)+10, y(i)+10, num2str(i-1), 'FontSize', 10);
end

% 绘制各无人机避障路径
legend_entries = {'禁飞区', '配送站', '目的地'};
for drone = 1:num_drones
    if isempty(drone_paths{drone})
        continue;
    end
    path = drone_paths{drone};
    for i = 1:length(path)-1
        curr = path(i);
        next_n = path(i+1);
        p = path_storage{curr, next_n};
        plot(p(:,1), p(:,2), 'Color', colors(drone,:), 'LineWidth', 2);
    end
    legend_entries{end+1} = ['无人机' num2str(drone) '路径'];
end

title('多无人机配送路径规划结果（A*避障）', 'FontSize', 14);
xlabel('X坐标 (米)', 'FontSize', 12);
ylabel('Y坐标 (米)', 'FontSize', 12);
grid on;
legend(legend_entries);
axis equal;

% ====================== 结果输出 ======================
fprintf('单无人机最优路径长度: %.2f 米\n', best_dist);
fprintf('多无人机总飞行距离: %.2f 米\n', sum(drone_dists));