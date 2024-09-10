% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% Debug code for initialize_particles function
PF = particle_filter_functions();
ST = state_transition_functions();
%M = measurement_functions();
% Set up test parameters
% State transition Parameters
params = struct();
% Simulation parameters
params.dt = 1;  % time step (s)
params.num_iterations = 40; % Number of time steps
params.num_vehicles = 1; % Number of vehicles 
params.num_particles = 5;
% generate_traffic_signal_states
params.red_time = 8;   % Red light duration (s)
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
params.d_loop1 = 150;
params.d_loop2 = 200;
params.d_loop3 = 250;
params.dt_loop1 = 1; % s
params.dt_loop2 = 1; % s
params.dt_loop3 = 1;
% s
params.accuracy_loop2 = 0.95;

% Floating sensors
params.sigma_GPS = 5;

%% generate ground truth states

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
[all_states, ~] = simulate1(initial_states, params);
%% generate ground truth states, plotting

% Prepare data for plotting
num_vehicles = params.num_vehicles;
num_iterations = params.num_iterations;
time_steps_vehicle = (0:params.num_iterations -1) * params.dt;
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

% 创建一个新图形
figure(1);
hold on;
figure('Position', [100, 100, 1200, 900]);  % Resize figure to fit subplot

% 1. Vehicle location and traffic signal status
subplot(2, 2, 1);
hold on;
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
for k = 1:length(S)
    color = color_map(S(k));
    line([time_steps_signal(k) time_steps_signal(k)], [params.d_stop_line params.d_stop_line + 5], 'Color', color, 'LineWidth', 2);
    %plot(time_steps_signal(k), params.d_stop_line, [color, '.'], 'MarkerSize', 5);
end
for i = 1:num_vehicles
    plot(time_steps_vehicle, positions(:, i), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
end
yline(params.d_loop1, 'r--', 'LineWidth', 1);
yline(params.d_loop2, 'b--', 'LineWidth', 1);
yline(params.d_loop3, 'g--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Positions and Traffic Signals');
ylim([0, params.d_stop_line + 50]);
xlim([0, max(time_steps_vehicle(end), time_steps_signal(end))]);
legend_elements = [plot(NaN, NaN, 'r.', 'MarkerSize', 20), ...
                   plot(NaN, NaN, 'y.', 'MarkerSize', 20), ...
                   plot(NaN, NaN, 'g.', 'MarkerSize', 20), ...
                   plot(NaN, NaN, 'r--', 'LineWidth', 1), ...
                   plot(NaN, NaN, 'b--', 'LineWidth', 1), ...
                   plot(NaN, NaN, 'g--', 'LineWidth', 1), ...
                   plot(NaN, NaN, '-', 'Color', 'b')];
legend(legend_elements, {'Red Signal', 'Yellow Signal', 'Green Signal', ...
                         'Vehicle Count Loop', 'Vehicle Presence Loop', 'Vehicle Speed Loop', 'Vehicle'}, ...
                         'Location', 'best');
grid on;

% 2. Vehicle speedpresencePresence
subplot(2, 2, 2);
hold on;
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
for k = 1:length(S)
    color = color_map(S(k));
    line([time_steps_signal(k) time_steps_signal(k)], [-0.7 -0.3], 'Color', color, 'LineWidth', 2);
    %plot(time_steps_signal(k), -0.5, [color, '|'], 'MarkerSize', 5);
end
for v = 1:num_vehicles
    plot(time_steps_vehicle, velocities(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
end
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vehicle Velocities');
legend('Location', 'best');
grid on;

% 3. Vehicle acceleration
subplot(2, 2, 3);
hold on;
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
for k = 1:length(S)
    color = color_map(S(k));
    line([time_steps_signal(k) time_steps_signal(k)], [-1.65 -1.6], 'Color', color, 'LineWidth', 2);
    %plot(time_steps_signal(k), -1.6, [color, '|'], 'MarkerSize', 5);
end
for v = 1:num_vehicles
    plot(time_steps_vehicle, accelerations(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
end
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Vehicle Accelerations');
legend('Location', 'best');
grid on;

% 4. Vehicle Decision
subplot(2, 2, 4);
hold on;
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
for k = 1:length(S)
    color = color_map(S(k));
    line([time_steps_signal(k) time_steps_signal(k)], [0.85 0.9], 'Color', color, 'LineWidth', 2);
    %plot(time_steps_signal(k), 0.9, [color, '|'], 'MarkerSize', 5);
end
for v = 1:num_vehicles
    plot(time_steps_vehicle , decisions(:, v), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', v));
end
xlabel('Time (s)');
ylabel('Decision');
title('Vehicle Decisions');
yticks([1, 2, 3]);
yticklabels({'Stop', 'Go', 'Undecided'});
legend('Location', 'best');
grid on;

% Adjust the spacing between subplots
spacing = 0.05;
subplot_pos = get(subplot(2,2,1), 'Position');
set(subplot(2,2,1), 'Position', [subplot_pos(1), subplot_pos(2), subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
subplot_pos = get(subplot(2,2,2), 'Position');
set(subplot(2,2,2), 'Position', [subplot_pos(1)+spacing, subplot_pos(2), subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
subplot_pos = get(subplot(2,2,3), 'Position');
set(subplot(2,2,3), 'Position', [subplot_pos(1), subplot_pos(2)+spacing, subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
subplot_pos = get(subplot(2,2,4), 'Position');
set(subplot(2,2,4), 'Position', [subplot_pos(1)+spacing, subplot_pos(2)+spacing, subplot_pos(3)-spacing, subplot_pos(4)-spacing]);

% Add a general title
sgtitle('Ground Truth States', 'FontSize', 16);

% Ask the user whether to save the graphic
save_choice = input('Do you want to save the figure? (y/n): ', 's');
if strcmpi(save_choice, 'y')
    % Create a folder to save the results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\1111', char(current_time));
    mkdir(folder_path);
    
    % Save the drawing
    saveas(gcf, fullfile(folder_path, 'groundTruth_States_Measurements.fig'));
    saveas(gcf, fullfile(folder_path, 'groundTruth_States_Measurements.eps'));
    saveas(gcf, fullfile(folder_path, 'groundTruth_States_Measurements.png'));
    
    disp(['The figure has been saved in the folder: ' folder_path]);
else
    disp('Figure was not saved.');
end

%% plotting ground truth measurements
% ... [Keep all the existing code up to the plotting section] ...

% Calculate ground truth measurements
M = measurement_functions();
c = zeros(1, num_iterations);
c_tilde = zeros(1, num_iterations);
o = zeros(1, num_iterations);
o_tilde = zeros(1, num_iterations);
v_avg = zeros(1, num_iterations);
v_avg_tilde = zeros(1, num_iterations);
d_tilde = zeros(num_vehicles, num_iterations);

for t = 1:num_iterations
    % Count vehicles passing the loop
    c(t) = M.count_loop(params, positions(t, :), velocities(t, :));
    % Measure the count with added noise
    c_tilde(t) = M.measure_c(c(t));
    
    % Presence loop
    o(t) = M.presence_loop(params, positions(t, :), velocities(t, :));
    o_tilde(t) = M.measure_o(o(t), params);
    
    % Speed loop
    v_avg(t) = M.speed_loop(params, positions(t, :), velocities(t, :));
    v_avg_tilde(t) = M.measure_v_avg(v_avg(t));
    
    % GPS
    for i = 1:num_vehicles
        d_tilde(i, t) = M.measure_d(positions(t, i), params);
    end
end

% Create a new figure for measurements
figure(2);
figure('Position', [100, 100, 1200, 900]);

% 1. Vehicle count
subplot(2, 2, 1);
hold on;
stairs(time_steps_vehicle, c, 'b-', 'LineWidth', 2, 'DisplayName', 'True Count');
stairs(time_steps_vehicle, c_tilde, 'r', 'LineWidth', 1, 'DisplayName', 'Measured Count');
xlabel('Time (s)');
ylabel('Vehicle Count');
title('Vehicle Count Measurement');
legend('Location', 'best');
grid on;

% 2. Vehicle presence
subplot(2, 2, 2);
hold on;
stairs(time_steps_vehicle, o, 'b-', 'LineWidth', 2, 'DisplayName', 'True Presence');
stairs(time_steps_vehicle, o_tilde, 'r', 'LineWidth', 1, 'DisplayName', 'Measured Presence');
xlabel('Time (s)');
ylabel('Vehicle Presence');
title('Vehicle Presence Measurement');
legend('Location', 'best');
grid on;

% 3. Average velocity
subplot(2, 2, 3);
hold on;
plot(time_steps_vehicle, v_avg, 'b-', 'LineWidth', 2, 'DisplayName', 'True Avg Velocity');
plot(time_steps_vehicle, v_avg_tilde, 'r', 'LineWidth', 1, 'DisplayName', 'Measured Avg Velocity');
xlabel('Time (s)');
ylabel('Average Velocity (m/s)');
title('Average Velocity Measurement');
legend('Location', 'best'); 
grid on;

% 4. GPS position measurement
subplot(2, 2, 4);
hold on;
for v = 1:num_vehicles
    plot(time_steps_vehicle, positions(:, v), 'b-', 'LineWidth', 2, 'DisplayName', sprintf('True Position %d', v));
    plot(time_steps_vehicle, d_tilde(v, :), 'r', 'LineWidth', 1, 'DisplayName', sprintf('Measured Position %d', v));
end
xlabel('Time (s)');
ylabel('Position (m)');
title('GPS Position Measurement');
legend('Location', 'best');
grid on;

% Adjust the spacing between subplots
spacing = 0.05;
for i = 1:4
    subplot_pos = get(subplot(2,2,i), 'Position');
    set(subplot(2,2,i), 'Position', [subplot_pos(1)+(mod(i,2)*spacing), subplot_pos(2)+floor((i-1)/2)*spacing, subplot_pos(3)-spacing, subplot_pos(4)-spacing]);
end

% Add a general title
sgtitle('Ground Truth Measurements', 'FontSize', 16);


% Ask the user whether to save the graphic
save_choice = input('Do you want to save the measurements figure? (y/n): ', 's');
if strcmpi(save_choice, 'y')
    % Use the same folder path as before
    saveas(gcf, fullfile(folder_path, 'groundTruth_Measurements.fig'));
    saveas(gcf, fullfile(folder_path, 'groundTruth_Measurements.eps'));
    saveas(gcf, fullfile(folder_path, 'groundTruth_Measurements.png'));
    
    disp(['The measurements figure has been saved in the folder: ' folder_path]);
else
    disp('Measurements figure was not saved.');
end
%% %% Validate decision making model (single vehicle)

PLOT = plotting_functions();
params.num_vehicles = 1;
params.num_particles = 5;

% Generate ground truth
[ground_truth_states, all_signals] = PLOT.generate_ground_truth(params, ST, PF);

% Generate measurements
M = measurement_functions();
ground_truth_measurements = PLOT.generate_measurements;
disp(size(ground_truth_measurements));

% Initialize particles
init_bounds = [0, params.D_h;
               params.v_desired - 1, params.v_desired;
               -params.b_max, params.a_max;
               1, 3];
particles = PF.initialize_particles(init_bounds, params);

disp('Checking variables before calling run_particle_filter:');
disp(['particles: ', num2str(size(particles))]);
disp(['ground_truth_measurements: ', num2str(size(ground_truth_measurements))]);
disp(['all_signals: ', num2str(size(all_signals))]);
disp(['params: ', num2str(length(fieldnames(params)))]);
disp('PF, ST, M structures exist');

% Run particle filter
[estimated_states, particle_trajectories, weights] = PLOT.run_particle_filter(particles, ground_truth_measurements, all_signals, params, PF, ST, M);


% Calculate MAE
mae = PLOT.calculate_mae(estimated_states, ground_truth_states);
if isnumeric(mae)
    disp(['Mean Absolute Error: ', num2str(mae)]);
else
    disp('Error: MAE calculation did not return a numeric value');
    disp(['MAE type: ', class(mae)]);
    disp('MAE value:');
    disp(mae);
end

figure(3);
figure('Position', [100, 100, 1200, 900]);
state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};

for i = 1:4
    subplot(2, 2, i);
    hold on;
    
    % 1. Plot ground truth in green
    plot(1:params.num_iterations, squeeze(ground_truth_states(:, 1, i)), 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    
    % 2 & 4. Plot all particle trajectories with weight-based thickness
    for j = 1:params.num_particles
        % Calculate the average weight for this particle
        avg_weight = mean(weights(:, j));
        % Normalize the weight to get a reasonable line width
        line_width = 0.1 + 5 * avg_weight / max(mean(weights, 1));
        plot(1:params.num_iterations, squeeze(particle_trajectories(:, j, 1, i)), 'Color', [0.3, 0.3, 0.3, 0.3], 'LineWidth', line_width);
    end
    
    % 3. Plot weighted estimated trajectory in blue
    plot(1:params.num_iterations, squeeze(estimated_states(:, 1, i)), 'b-', 'LineWidth', 2, 'DisplayName', 'Estimated');
    
    title(state_names{i});
    xlabel('Time step');
    ylabel(state_names{i});
    legend('Location', 'best');
    grid on;
    
    % Add traffic signal information
    if i == 1  % Only add to position plot
        color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
        for t = 1:params.num_iterations
            color = color_map(all_signals{t});
            plot(t, params.d_stop_line, [color, '.'], 'MarkerSize', 10);
        end
        yline(params.d_loop1, 'r--', 'LineWidth', 1, 'DisplayName', 'Vehicle Count Loop');
    end
end

sgtitle('Decision Making Model Validation (Single Vehicle)', 'FontSize', 16);

% Ask the user whether to save the graphic
save_choice = input('Do you want to save the figure? (y/n): ', 's');
if strcmpi(save_choice, 'y')
    % Create a folder to save the results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\1111', char(current_time));
    mkdir(folder_path);
    
    % Save the drawing
    saveas(gcf, fullfile(folder_path, 'Decision Making Model.fig'));
    saveas(gcf, fullfile(folder_path, 'Decision Making Model.eps'));
    saveas(gcf, fullfile(folder_path, 'Decision Making Model.png'));
    
    disp(['The figure has been saved in the folder: ' folder_path]);
else
    disp('Figure was not saved.');
end
close all;

