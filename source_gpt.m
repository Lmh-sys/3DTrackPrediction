% 定义场景大小
width = 100;
height = 100;

% 定义机器人数量和位置
num_robots = 5;
robot_positions = randi([1 width], num_robots, 2);

% 定义放射源位置
source_position = randi([1 width], 1, 2);

% 定义障碍物数量和位置
num_obstacles = 10;
obstacle_positions = randi([1 width], num_obstacles, 2);

% 绘制场景
figure;
hold on;

% 绘制机器人
for i = 1:num_robots
    plot(robot_positions(i,1), robot_positions(i,2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
end

% 绘制放射源
plot(source_position(1), source_position(2), 'r*', 'MarkerSize', 20, 'LineWidth', 2);

% 绘制障碍物
for i = 1:num_obstacles
    rectangle('Position', [obstacle_positions(i,1)-1, obstacle_positions(i,2)-1, 2, 2], 'FaceColor', 'k', 'LineWidth', 1);
end

% 设置坐标轴和标题
xlim([1 width]);
ylim([1 height]);
xlabel('X');
ylabel('Y');
title('Multi-Robot Search for Radiation Sources with Obstacles');

% 显示图像
hold off;
