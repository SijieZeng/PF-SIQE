% 设置参数
params.num_iterations = 22;  % 总的迭代次数
params.num_vehicles = 5;
params.yellow_time = 3.5;
params.delta_t = 0.5;
params.dt = 1;
params.red_time = 1;
params.green_time = 1;
params.d_stop_line = 300;

StateTransition = state_transition();

% 生成交通信号状态
S = StateTransition.generate_traffic_signal_states(params);

% 创建图形
figure(1);
hold on;

% 设置颜色映射
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});

% 绘制交通信号状态
for k = 1:length(S)
    time = (k - 1) * params.delta_t;
    color = color_map(S(k));
    plot(time, params.d_stop_line, [color, '.'], 'MarkerSize', 20);
end

% 设置图形属性
xlabel('Time (s)');
ylabel('Distance from Stop Line (m)');
title('Traffic Signal States Over Time');
ylim([0 params.d_stop_line + 50]);
xlim([0 (params.num_iterations - 1) * params.delta_t]);

% 添加图例
legend_elements = [
    plot(NaN, NaN, 'r.', 'MarkerSize', 20);
    plot(NaN, NaN, 'y.', 'MarkerSize', 20);
    plot(NaN, NaN, 'g.', 'MarkerSize', 20)
];
legend(legend_elements, {'Red', 'Yellow', 'Green'}, 'Location', 'best');

% 添加网格
grid on;

hold off;

% 打印状态序列
disp('Traffic Signal State Sequence:');
disp(S');