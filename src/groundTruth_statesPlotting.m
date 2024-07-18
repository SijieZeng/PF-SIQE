% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% Increase graphics timeout
set(0, 'DefaultFigureCreateFcn', @(fig, ~)set(fig, 'CloseRequestFcn', @(src, ~)close(src)));

% Debug code for initialize_particles function
PF = particle_filter_functions();
ST = state_transition_functions();
M = measurement_functions();
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
params.d_loop1 = 200;
params.d_loop2 = 200;
params.d_loop3 = 200;
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
[all_states, all_signals] = simulate1(initial_states, params);
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
figure('Position', [100, 100, 1200, 900]);  % Resize figure to fit subplot

% 1. Vehicle location and traffic signal status
subplot(2, 2, 1);
hold on;
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
for k = 1:length(S)
    color = color_map(S(k));
    plot(time_steps_signal(k), params.d_stop_line, [color, '.'], 'MarkerSize', 20);
end
for i = 1:num_vehicles
    plot(time_steps_vehicle, positions(:, i), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
end
xlabel('Time (s)');
ylabel('Position (m)');
title('Vehicle Positions and Traffic Signals');
ylim([0, params.d_stop_line + 50]);
xlim([0, max(time_steps_vehicle(end), time_steps_signal(end))]);
legend_elements = [plot(NaN,NaN,'r.','MarkerSize',20), plot(NaN,NaN,'y.','MarkerSize',20), plot(NaN,NaN,'g.','MarkerSize',20)];
legend([legend_elements, plot(NaN,NaN,'-','Color','b')], {'Red', 'Yellow', 'Green', 'Vehicle'}, 'Location', 'best');
grid on;

% 2. Vehicle speed
subplot(2, 2, 2);
hold on;
color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
for k = 1:length(S)
    color = color_map(S(k));
    plot(time_steps_signal(k), -0.5, [color, '.'], 'MarkerSize', 20);
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
    plot(time_steps_signal(k), -1.6, [color, '.'], 'MarkerSize', 20);
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
    plot(time_steps_signal(k), 0.9, [color, '.'], 'MarkerSize', 20);
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
sgtitle('Groundtruth States', 'FontSize', 16);

% Ask the user whether to save the graphic
save_choice = input('Do you want to save the figure? (y/n): ', 's');
if strcmpi(save_choice, 'y')
    % Create a folder to save the results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    folder_path = fullfile('C:\Users\Sijie\Documents\MATLAB\ParticleFilter-simple-case\results\groundtruth_states', char(current_time));
    mkdir(folder_path);
    
    % Save the drawing
    saveas(gcf, fullfile(folder_path, 'groundtruth_states.fig'));
    saveas(gcf, fullfile(folder_path, 'groundtruth_states.eps'));
    saveas(gcf, fullfile(folder_path, 'groundtruth_states.png'));
    
    disp(['The figure has been saved in the folder: ' folder_path]);
else
    disp('Figure was not saved.');
end

close all;
%% generate state-based prediction output 



