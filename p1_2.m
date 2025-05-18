clc, clear, close all

% 坐标数据（示例：26个点，包含配送站0号）
coordinates = [  
    120.719393,30.736683;   % 0号点
    120.723462, 30.738040;  % 1号点
    120.723064, 30.737176;  % 2号点
    120.724139, 30.738074;  % 3号点
    120.720783, 30.736567;  % 4号点
    120.719332, 30.738759;  % 5号点
    120.717844, 30.736583;  % 6号点
    120.723751, 30.737023;  % 7号点
    120.723722, 30.737300;  % 8号点
    120.721873, 30.738392;  % 9号点
    120.723267, 30.738555;  % 10号点
    120.715796, 30.739321;  % 11号点
    120.716573, 30.739419;  % 12号点
    120.716315, 30.738643;  % 13号点
    120.713438, 30.739864;  % 14号点
    120.712416, 30.739130;  % 15号点
    120.710881, 30.737107;  % 16号点
    120.708420, 30.738185;  % 17号点
    120.707961, 30.737138;  % 18号点
    120.707201, 30.738079;  % 19号点
    120.709221, 30.737340;  % 20号点
    120.707649, 30.736115;  % 21号点
    120.709162, 30.736004;  % 22号点
    120.709125, 30.736497;  % 23号点
    120.709164, 30.736937;  % 24号点
    120.710817, 30.738949   % 25号点
];
%% 问题1（2）改进版：基于装载量约束的多无人机任务分配
% 输入：节点经纬度坐标、总装载量、单架无人机最大装载量
% 输出：满足装载量约束的最优路径及总距离

% 计算距离矩阵（基于经纬度的实际距离，单位：米）
n = size(coordinates, 1);
distance_matrix = zeros(n, n);
earth_radius = 6371000;  % 地球半径（米）

for i = 1:n
    for j = i+1:n
        lat1 = coordinates(i, 2) * pi / 180;
        lon1 = coordinates(i, 1) * pi / 180;
        lat2 = coordinates(j, 2) * pi / 180;
        lon2 = coordinates(j, 1) * pi / 180;
        
        % Haversine公式计算两点间距离
        dlat = lat2 - lat1;
        dlon = lon2 - lon1;
        a = sin(dlat/2)^2 + cos(lat1) * cos(lat2) * sin(dlon/2)^2;
        c = 2 * atan2(sqrt(a), sqrt(1-a));
        distance_matrix(i, j) = earth_radius * c;
        distance_matrix(j, i) = distance_matrix(i, j);
    end
end

% 设置总装载量和单架无人机最大装载量（可调整）
total_load = 100;
drone_capacity = 35;  % 单架无人机最大装载量

% 为每个目的地随机分配装载量（1-5之间）
node_loads = randi([1, 5], n-1, 1);  % 节点1-25的装载量
node_loads = [0; node_loads];  % 节点0（配送站）装载量为0

% 计算需要的无人机数量（向上取整）
num_drones = ceil(total_load / drone_capacity);
fprintf('总装载量: %d, 单架无人机装载量: %d, 需要无人机数量: %d\n', total_load, drone_capacity, num_drones);

% 基于装载量约束的分组算法
% 目标：每组总装载量尽量均衡且不超过无人机容量
groups = cell(num_drones, 1);
group_loads = zeros(num_drones, 1);
remaining_nodes = 2:n;  % 排除配送站

% 贪心分组算法：按装载量降序排列节点，依次分配给当前装载量最小的组
[~, sort_idx] = sort(node_loads(remaining_nodes), 'descend');
sorted_nodes = remaining_nodes(sort_idx);

for i = 1:length(sorted_nodes)
    node = sorted_nodes(i);
    % 找到当前装载量最小且能容纳该节点的组
    [~, min_idx] = min(group_loads);
    valid_groups = find(group_loads + node_loads(node) <= drone_capacity);
    
    if ~isempty(valid_groups)
        % 优先选择能容纳且当前装载量最小的组
        [~, idx] = min(group_loads(valid_groups));
        group_idx = valid_groups(idx);
    else
        % 如果没有组能容纳，选择当前装载量最小的组
        group_idx = min_idx;
    end
    
    % 添加节点到选中的组
    if isempty(groups{group_idx})
        groups{group_idx} = [1; node];  % 添加配送站和当前节点
    else
        groups{group_idx} = [groups{group_idx}; node];
    end
    group_loads(group_idx) = group_loads(group_idx) + node_loads(node);
end

% 输出分组结果（修复后的代码）
fprintf('\n分组结果:\n');
for i = 1:num_drones
    % 将数值数组转换为字符串元胞数组
    node_strings = cellstr(num2str(groups{i}'));
    fprintf('无人机 %d: 节点 = [%s], 装载量 = %d/%d\n', i, ...
            strjoin(node_strings, ', '), group_loads(i), drone_capacity);
end

% 蚁群算法参数设置
num_ants = 50;            % 蚂蚁数量
max_iterations = 200;     % 最大迭代次数
alpha = 1;                % 信息素重要程度因子
beta = 2;                 % 启发式信息重要程度因子
rho = 0.5;                % 信息素挥发因子
Q = 100;                  % 信息素增加强度系数

% 为每架无人机求解TSP子问题
drone_paths = cell(num_drones, 1);
drone_distances = zeros(num_drones, 1);

figure(2);
hold on;
colors = lines(num_drones);

for i = 1:num_drones
    % 提取当前无人机的节点坐标和距离矩阵
    nodes = groups{i};
    sub_coordinates = coordinates(nodes, :);
    sub_n = length(nodes);
    sub_distance_matrix = zeros(sub_n, sub_n);
    
    for j = 1:sub_n
        for k = j+1:sub_n
            sub_distance_matrix(j, k) = distance_matrix(nodes(j), nodes(k));
            sub_distance_matrix(k, j) = sub_distance_matrix(j, k);
        end
    end
    
    % 使用蚁群算法求解子TSP问题
    sub_pheromone = ones(sub_n, sub_n);
    for j = 1:sub_n
        sub_pheromone(j, j) = 0;
    end
    
    sub_best_path = [];
    sub_best_distance = inf;
    
    for iter = 1:max_iterations
        sub_ant_paths = zeros(num_ants, sub_n);
        sub_ant_distances = zeros(num_ants, 1);
        
        for ant = 1:num_ants
            visited = zeros(1, sub_n);
            current_node = 1;  % 从配送站开始
            visited(current_node) = 1;
            sub_ant_paths(ant, 1) = current_node;
            
            for j = 2:sub_n
                unvisited = find(visited == 0);
                probabilities = zeros(size(unvisited));
                
                for k = 1:length(unvisited)
                    probabilities(k) = sub_pheromone(current_node, unvisited(k))^alpha * ...
                                     (1/sub_distance_matrix(current_node, unvisited(k)))^beta;
                end
                
                if sum(probabilities) == 0
                    next_node = unvisited(1);
                else
                    probabilities = probabilities / sum(probabilities);
                    cum_prob = cumsum(probabilities);
                    r = rand;
                    next_node_idx = find(cum_prob >= r, 1, 'first');
                    next_node = unvisited(next_node_idx);
                end
                
                current_node = next_node;
                visited(current_node) = 1;
                sub_ant_paths(ant, j) = current_node;
            end
            
            total_distance = 0;
            for j = 1:sub_n-1
                total_distance = total_distance + sub_distance_matrix(sub_ant_paths(ant, j), sub_ant_paths(ant, j+1));
            end
            total_distance = total_distance + sub_distance_matrix(sub_ant_paths(ant, sub_n), sub_ant_paths(ant, 1));
            sub_ant_distances(ant) = total_distance;
            
            if total_distance < sub_best_distance
                sub_best_distance = total_distance;
                sub_best_path = sub_ant_paths(ant, :);
            end
        end
        
        % 更新信息素
        sub_pheromone = (1 - rho) * sub_pheromone;
        
        for ant = 1:num_ants
            for j = 1:sub_n-1
                sub_pheromone(sub_ant_paths(ant, j), sub_ant_paths(ant, j+1)) = ...
                    sub_pheromone(sub_ant_paths(ant, j), sub_ant_paths(ant, j+1)) + Q / sub_ant_distances(ant);
                sub_pheromone(sub_ant_paths(ant, j+1), sub_ant_paths(ant, j)) = ...
                    sub_pheromone(sub_ant_paths(ant, j+1), sub_ant_paths(ant, j));
            end
            sub_pheromone(sub_ant_paths(ant, sub_n), sub_ant_paths(ant, 1)) = ...
                sub_pheromone(sub_ant_paths(ant, sub_n), sub_ant_paths(ant, 1)) + Q / sub_ant_distances(ant);
            sub_pheromone(sub_ant_paths(ant, 1), sub_ant_paths(ant, sub_n)) = ...
                sub_pheromone(sub_ant_paths(ant, 1), sub_ant_paths(ant, sub_n));
        end
    end
    
    % 转换为全局节点编号
    global_path = nodes(sub_best_path);
    drone_paths{i} = global_path;
    drone_distances(i) = sub_best_distance;
    
    % 可视化当前无人机路径
    plot(sub_coordinates(:,1), sub_coordinates(:,2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
    plot(sub_coordinates(sub_best_path,1), sub_coordinates(sub_best_path,2), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    plot([sub_coordinates(sub_best_path(end),1), sub_coordinates(sub_best_path(1),1)], ...
         [sub_coordinates(sub_best_path(end),2), sub_coordinates(sub_best_path(1),2)], '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    
    % 添加节点编号和装载量标注
    for j = 1:sub_n
        node_idx = nodes(j);
        text(sub_coordinates(j,1)+0.0002, sub_coordinates(j,2)+0.0002, ...
             [num2str(node_idx-1), '(', num2str(node_loads(node_idx)), ')']);
    end
    
    % 输出结果
    fprintf('\n无人机 %d 路径（节点编号）: ', i);
    fprintf('%d -> ', global_path);
    fprintf('%d\n', global_path(1));
    fprintf('无人机 %d 飞行距离: %.2f 米, 装载量: %d/%d\n', ...
            i, sub_best_distance, group_loads(i), drone_capacity);
end

% 绘制配送站
plot(coordinates(1,1), coordinates(1,2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(coordinates(1,1)+0.0002, coordinates(1,2)+0.0002, '0(0)');

title('基于装载量约束的多无人机最优路径');
xlabel('经度');
ylabel('纬度');
grid on;
axis equal;
legend_str = cell(num_drones, 1);
for i = 1:num_drones
    legend_str{i} = sprintf('无人机%d(载重%d/%d)', i, group_loads(i), drone_capacity);
end
legend(legend_str);

% 计算总距离
total_distance = sum(drone_distances);
fprintf('\n所有无人机总飞行距离: %.2f 米\n', total_distance);