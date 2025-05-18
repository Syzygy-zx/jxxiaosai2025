clc, clear, close all
%% 问题1（1）：单架无人机路径优化（TSP问题）
% 输入：节点经纬度坐标
% 输出：最优路径及总距离

% 经纬度坐标数据（26个节点，包括配送站0和25个目的地）
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

% 蚁群算法参数设置
num_ants = 50;            % 蚂蚁数量
max_iterations = 200;     % 最大迭代次数
alpha = 1;                % 信息素重要程度因子
beta = 2;                 % 启发式信息重要程度因子
rho = 0.5;                % 信息素挥发因子
Q = 100;                  % 信息素增加强度系数

% 初始化信息素矩阵
pheromone_matrix = ones(n, n);
for i = 1:n
    pheromone_matrix(i, i) = 0;  % 对角线为0，避免自环
end

% 初始化最优路径和距离
best_path = [];
best_distance = inf;

% 蚁群算法主循环
for iter = 1:max_iterations
    % 存储每只蚂蚁的路径和距离
    ant_paths = zeros(num_ants, n);
    ant_distances = zeros(num_ants, 1);
    
    % 每只蚂蚁构建路径
    for ant = 1:num_ants
        % 初始化已访问节点集合
        visited = zeros(1, n);
        current_node = 1;  % 从配送站（节点0）开始
        visited(current_node) = 1;
        ant_paths(ant, 1) = current_node;
        
        % 依次访问剩余节点
        for i = 2:n
            % 计算未访问节点的概率
            unvisited = find(visited == 0);
            probabilities = zeros(size(unvisited));
            
            for j = 1:length(unvisited)
                probabilities(j) = pheromone_matrix(current_node, unvisited(j))^alpha * ...
                                 (1/distance_matrix(current_node, unvisited(j)))^beta;
            end
            
            % 轮盘赌选择下一个节点
            if sum(probabilities) == 0
                next_node = unvisited(1);
            else
                probabilities = probabilities / sum(probabilities);
                cum_prob = cumsum(probabilities);
                r = rand;
                next_node_idx = find(cum_prob >= r, 1, 'first');
                next_node = unvisited(next_node_idx);
            end
            
            % 更新当前节点和访问状态
            current_node = next_node;
            visited(current_node) = 1;
            ant_paths(ant, i) = current_node;
        end
        
        % 计算路径总距离
        total_distance = 0;
        for i = 1:n-1
            total_distance = total_distance + distance_matrix(ant_paths(ant, i), ant_paths(ant, i+1));
        end
        % 返回配送站
        total_distance = total_distance + distance_matrix(ant_paths(ant, n), ant_paths(ant, 1));
        ant_distances(ant) = total_distance;
        
        % 更新最优路径
        if total_distance < best_distance
            best_distance = total_distance;
            best_path = ant_paths(ant, :);
        end
    end
    
    % 更新信息素矩阵
    pheromone_matrix = (1 - rho) * pheromone_matrix;
    
    for ant = 1:num_ants
        for i = 1:n-1
            pheromone_matrix(ant_paths(ant, i), ant_paths(ant, i+1)) = ...
                pheromone_matrix(ant_paths(ant, i), ant_paths(ant, i+1)) + Q / ant_distances(ant);
            pheromone_matrix(ant_paths(ant, i+1), ant_paths(ant, i)) = ...
                pheromone_matrix(ant_paths(ant, i+1), ant_paths(ant, i));
        end
        % 从最后一个节点返回起点
        pheromone_matrix(ant_paths(ant, n), ant_paths(ant, 1)) = ...
            pheromone_matrix(ant_paths(ant, n), ant_paths(ant, 1)) + Q / ant_distances(ant);
        pheromone_matrix(ant_paths(ant, 1), ant_paths(ant, n)) = ...
            pheromone_matrix(ant_paths(ant, 1), ant_paths(ant, n));
    end
    
    % 输出迭代信息
    if mod(iter, 10) == 0
        fprintf('迭代次数: %d, 最优距离: %.2f 米\n', iter, best_distance);
    end
end

% 输出结果
fprintf('\n单架无人机最优路径（节点编号）: ');
fprintf('%d -> ', best_path);
fprintf('%d\n', best_path(1));  % 返回起点
fprintf('总飞行距离: %.2f 米\n', best_distance);

% 可视化路径
figure(1);
plot(coordinates(:,1), coordinates(:,2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
hold on;
plot(coordinates(best_path,1), coordinates(best_path,2), 'r-', 'LineWidth', 1.5);
% 连接最后一个节点和起点
plot([coordinates(best_path(end),1), coordinates(best_path(1),1)], ...
     [coordinates(best_path(end),2), coordinates(best_path(1),2)], 'r-', 'LineWidth', 1.5);
plot(coordinates(best_path(1),1), coordinates(best_path(1),2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% 添加节点编号
for i = 1:n
    text(coordinates(i,1)+0.0002, coordinates(i,2)+0.0002, num2str(i-1));
end

title('单架无人机最优路径');
xlabel('经度');
ylabel('纬度');
grid on;
axis equal;
