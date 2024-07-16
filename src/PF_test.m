% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% Debug code for initialize_particles function
PF = particle_filter_functions();
ST = state_transition();
M = measurement();
% Set up test parameters
% State transition Parameters
params = struct();
% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 40; % Number of time steps
params.num_vehicles = 3; % Number of vehicles 
params.num_particles = 5;

% generate_traffic_signal_states
params.red_time = 5;   % Red light duration (s)
params.yellow_time = 3.5; % Yellow light duration (s), cannot tune by now
params.green_time = 5; % Green light duration (s)
params.delta_t = 0.5;
params.d_stop_line = 300; % Stop line position (m)               
params.lane = 1;

% decision making
params.D_h = params.d_stop_line - 150;       % Decision distance in meters
params.T_reaction = 1.0;% Reaction time in seconds
params.p_stop = 0.5;    % Probability of stopping in the dilemma zone
params.b_max = 1.5; 
params.d_0 = 0; 

% traffic light decision model
params.T_discrete = 1.0; 
params.a = 1.0;
params.D_stop = 1;  % Assuming 1 represents D_stop
params.D_go = 2;  % Assuming 2 represents D_go
params.D_undecided = 3; % Assuming 2 represents D_undecided

% IDM parameters
params.v_desired = 15;  % desired speed (m/s)
params.a_max = 1.0;     % maximum acceleration (m/s^2)
params.b = 1.5;         % comfortable deceleration (m/s^2)
params.s0 = 2.0;        % minimum gap (m)
params.vehicle_length = 5.0; %vehicle length (m)
params.T = 1.0;         % safe time headway (s)
params.delta = 4;       % acceleration exponent

% Acceleration noise
params.n_a = 0.1;  

% Measurement Parameters
% loop detectors
params.d_loop1 = 200;
params.d_loop2 = 200;
params.d_loop3 = 200;
params.dt_loop1 = 1; % s
params.dt_loop2 = 1; % s
params.dt_loop3 = 1; % s
params.accuracy_loop2 = 0.95;

% Floating sensors
params.sigma_GPS = 5;

%% 
init_bounds = [0, params.D_h;
               params.v_desired - 1, params.v_desired;
               - params.b_max, params.a_max];

% Call the function
particles = PF.initialize_particles(init_bounds, params);

% Calculate the mean state across all particles
mean_state = squeeze(mean(particles, 1));
if params.num_vehicles == 1
    mean_state = reshape(mean_state, [params.num_vehicles, size(particles, 3)]);
end

% Initialize state for each vehicle
initial_states = struct('d', {}, 'v', {}, 'a', {}, 'D', {});
for i = 1:params.num_vehicles
    initial_states(i).d = mean_state(i, 1);  % Position
    initial_states(i).v = mean_state(i, 2);  % Velocity
    initial_states(i).a = mean_state(i, 3);  % Acceleration
    initial_states(i).D = mean_state(i, 4);  % Decision
end


S = ST.generate_traffic_signal_states(params);

% Run simulation
[all_states, all_signals] = simulate1(initial_states, params);

% Prepare data for plotting
num_vehicles = params.num_vehicles;
num_iterations = params.num_iterations;
time_steps_vehicle = (0:params.num_iterations-1) * params.dt;
time_steps_signal = (0:length(S)-1) * params.delta_t;

% Initialize arrays to store states
positions = zeros(num_iterations, num_vehicles);
velocities = zeros(num_iterations, num_vehicles);
accelerations = zeros(num_iterations, num_vehicles);
decisions = zeros(num_iterations, num_vehicles);

% Extract data from all_states
for t = 1:num_iterations
    for i = 1:num_vehicles
        positions(t, i) = all_states{t}(i).d;
        velocities(t, i) = all_states{t}(i).v;
        accelerations(t, i) = all_states{t}(i).a;
        decisions(t, i) = all_states{t}(i).D;
    end
end

% 创建图形
figure(1);
hold on;

% 设置颜色映射
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});

% 绘制交通信号状态
for k = 1:length(S)
    color = color_map(S(k));
    plot(time_steps_signal(k), params.d_stop_line, [color, '.'], 'MarkerSize', 20);
end

for i = 1:num_vehicles
    plot(time_steps_vehicle, positions(:, i), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
end

% Customize the plot
xlabel('Time Step');
ylabel('Position (m)');
title('Vehicle Positions Over Time');
ylim([0, params.d_stop_line + 50]);
xlim([0, max(time_steps_vehicle(end), time_steps_signal(end))]);
legend_elements = [plot(NaN, NaN, 'r.', 'MarkerSize', 20), plot(NaN, NaN, 'y.', 'MarkerSize', 20), plot(NaN, NaN, 'g.', 'MarkerSize', 20)];
legend([legend_elements, plot(NaN, NaN, '-', 'Color', 'b')], {'Red', 'Yellow', 'Green', 'Vehicle'}, 'Location', 'best');
grid on;
hold off;


% Plot velocities (v)
figure(2);
hold on;
for v = 1:num_vehicles
    plot(time_steps_vehicle, velocities(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
end
xlabel('Time Step');
ylabel('Velocity (m/s)');
title('Vehicle Velocities Over Time');
legend('Location', 'best');
grid on;
hold off;

% Plot accelerations (a)
figure(3);
hold on;
for v = 1:num_vehicles
    plot(time_steps_vehicle, accelerations(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
end
xlabel('Time Step');
ylabel('Acceleration (m/s^2)');
title('Vehicle Accelerations Over Time');
legend('Location', 'best');
grid on;
hold off;

% Plot decisions (D)
figure(4);
hold on;
for v = 1:num_vehicles
    plot(time_steps_vehicle, decisions(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
end
xlabel('Time Step');
ylabel('Decision');
title('Vehicle Decisions Over Time');
yticks([1, 2, 3]);
yticklabels({'Stop', 'Go', 'Undecided'});
legend('Location', 'best');
grid on;
hold off;

% Display a prompt to the user asking if they want to save the figures
save_choice = input('Do you want to save the figures? (y/n): ', 's');

if strcmpi(save_choice, 'y')
    % Create a folder for saving results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    %folder_path = fullfile('/Users/celine/Documents/MATLAB/PF-SIQE_Msc_thesis/results', char(current_time));
    folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results', char(current_time));
    mkdir(folder_path);
    
    % Save figures
    figure_titles = {'Vehicle Position', 'Vehicle Speed', 'Vehicle Acceleration', 'Vehicle Decisions'};
    for fig_num = 1:4
        figure(fig_num);
        saveas(gcf, fullfile(folder_path, sprintf('%s_plot.fig', lower(strrep(figure_titles{fig_num}, ' ', '_')))));
    end
    
    disp(['The figures have been saved in the folder: ' folder_path]);
else
    disp('Figures were not saved.');
end
% Close all figures
close all;


% 设置参数
num_particles = size(particles, 1);
num_vehicles = size(particles, 2);
state_dim = size(particles, 3);
num_iterations = 15;

% 定义车辆形状
vehicle_shapes = {'o', 's', '^', 'v', '>', '<', 'p', 'h', 'd'};

% 为每个粒子分配一个随机颜色
particle_colors = rand(num_particles, 3);

% 创建一个图形窗口
figure('Position', [100, 100, 1200, 800]);

% 循环遍历每个状态维度
for state = 1:state_dim
    subplot(2, 2, state);
    hold on;
    
    % 绘制所有粒子的状态
    for p = 1:num_particles
        for v = 1:num_vehicles
            scatter(v, particles(p, v, state), 50, particle_colors(p,:), vehicle_shapes{mod(v-1, length(vehicle_shapes))+1}, 'filled');
        end
    end 
    
    % 计算并绘制每辆车的平均状态
    avg_states = squeeze(mean(particles(:,:,state), 1));
    scatter(1:num_vehicles, avg_states, 200, 'k', 'x', 'LineWidth', 3);
    
    % 设置图形属性
    title(sprintf('State %d Distribution', state));
    xlabel('Vehicle Index');
    ylabel(sprintf('State %d Value', state));
    xlim([0.5, num_vehicles+0.5]);
    grid on;
    
    % 添加图例
    if state == 1
        legend_entries = cell(1, num_vehicles + 1);
        for v = 1:num_vehicles
            legend_entries{v} = sprintf('Vehicle %d', v);
        end
        legend_entries{end} = 'Average';
        legend(legend_entries, 'Location', 'eastoutside');
    end
    
    hold off;
end

% 调整子图的布局
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);
sgtitle('Particle Filter State Distribution');

% 添加颜色条来表示粒子
colorbar_axes = axes('Position', [0.93, 0.1, 0.02, 0.8]);
colormap(particle_colors);
c = colorbar(colorbar_axes);
c.Label.String = 'Particle Index';
c.Label.Rotation = 270;
c.Label.VerticalAlignment = 'bottom';
clim([1, num_particles]);
axis off;