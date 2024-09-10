% Clear workspace, command window, and close all figuresinitialize_particles
clear; clc; close all;

% Load necessary functions
PF = particle_filter_functions();
ST = state_transition_functions();
M = measurement_functions();

% Set up parameters
params = setup_parameters();

%% 1. Validate decision making model (single vehicle)
params.num_vehicles = 1;
[ground_truth_states, ground_truth_measurements] = generate_ground_truth(params, ST, M);
plot_ground_truth(ground_truth_states, ground_truth_measurements, params);

% Initialize particles
init_bounds = [0, params.D_h;
               params.v_desired - 1, params.v_desired;
               -params.b_max, params.a_max];
particles = PF.initialize_particles(init_bounds, params);

% Run particle filter for decision making model validation
[estimated_states, particle_trajectories, weights] = run_particle_filter(particles, ground_truth_measurements, params, PF, ST, M);

% Plot results for decision making model validation
plot_decision_making_validation(ground_truth_states, estimated_states, particle_trajectories, weights, params);

%% 2. Validate particle filter (single vehicle with count loop)
% (Same code as above, but with different plotting function)
plot_particle_filter_validation(ground_truth_states, estimated_states, particle_trajectories, weights, params);

%% 3. Multiple vehicles (two vehicles)
params.num_vehicles = 2;
[ground_truth_states, ground_truth_measurements] = generate_ground_truth(params, ST, M);

% Initialize particles
particles = PF.initialize_particles(init_bounds, params);

% Run particle filter
[estimated_states, particle_trajectories, weights] = run_particle_filter(particles, ground_truth_measurements, params, PF, ST, M);

% Plot results for multiple vehicles
plot_multiple_vehicles(ground_truth_states, estimated_states, particle_trajectories, weights, params);

%% Calculate performance metrics
mae = calculate_mae(ground_truth_states, estimated_states);

%% Save results
save_results(ground_truth_states, estimated_states, particle_trajectories, weights, mae, params);

%% Helper functions
function params = setup_parameters()
    % Set up all parameters as in your original code
    params = struct();
    params.dt = 1;  % time step (s)
    params.num_iterations = 40; % Number of time steps
    params.num_particles = 5;
    params.num_vehicles = 1;
    params.red_time = 8;   % Red light duration (s)
    params.yellow_time = 3.5; % Yellow light duration (s)
    params.green_time = 5; % Green light duration (s)
    params.delta_t = 0.5;
    params.d_stop_line = 300; % Stop line position (m)               
    params.lane = 1;
    params.D_h = params.d_stop_line - 150;       % Decision distance in meters
    params.T_reaction = 1.0;% Reaction time in seconds
    params.p_stop = 0.5;    % Probability of stopping in the dilemma zone
    params.b_max = 1.5; 
    params.d_0 = 0; 
    params.T_discrete = 1.0; 
    params.a = 1.0;
    params.D_stop = 1;  % Assuming 1 represents D_stop
    params.D_go = 2;  % Assuming 2 represents D_go
    params.D_undecided = 3; % Assuming 3 represents D_undecided
    params.v_desired = 15;  % desired speed (m/s)
    params.a_max = 1.0;     % maximum acceleration (m/s^2)
    params.b = 1.5;         % comfortable deceleration (m/s^2)
    params.s0 = 2.0;        % minimum gap (m)
    params.vehicle_length = 5.0; %vehicle length (m)
    params.T = 1.0;         % safe time headway (s)
    params.delta = 4;       % acceleration exponent
    params.n_a = 0.1;  
    params.d_loop1 = 150;
    params.d_loop2 = 200;
    params.d_loop3 = 250;
    params.dt_loop1 = 1; % s
    params.dt_loop2 = 1; % s
    params.dt_loop3 = 1; % s
    params.accuracy_loop2 = 0.95;
    params.sigma_GPS = 5;
end
function [ground_truth_states, ground_truth_measurements] = generate_ground_truth(params, ST, M)
    % Generate traffic signal states
    S = ST.generate_traffic_signal_states(params);
    
    % Initialize states
    init_bounds = [0, params.D_h;
                   params.v_desired - 1, params.v_desired;
                   -params.b_max, params.a_max];
    PF = particle_filter_functions();
    initial_states = PF.initialize_particles(init_bounds, params);
    mean_state = squeeze(mean(initial_states, 1));


    % 确保 mean_state 是一个 num_vehicles x 4 的矩阵
    if size(mean_state, 2) == 1
        mean_state = mean_state';
    end

     % 将 mean_state 转换为结构体数组
    initial_states = struct('d', num2cell(mean_state(:, 1)), ...
                            'v', num2cell(mean_state(:, 2)), ...
                            'a', num2cell(mean_state(:, 3)), ...
                            'D', num2cell(mean_state(:, 4)));
    
    % Run simulation
    [all_states, all_signals] = simulate1(initial_states, params);

    
    % Extract ground truth states
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;
    ground_truth_states = zeros(num_iterations, num_vehicles, 4);
    for t = 1:num_iterations
        for i = 1:num_vehicles
            ground_truth_states(t, i, :) = [all_states{t}(i).d, all_states{t}(i).v, all_states{t}(i).a, all_states{t}(i).D];
        end
    end
    
    % Generate measurements
    ground_truth_measurements = generate_measurements(ground_truth_states, params, M);
end

function [estimated_states, particle_trajectories, weights] = run_particle_filter(particles, measurements, params, PF, ST, M)
    num_iterations = params.num_iterations;
    num_particles = params.num_particles;
    num_vehicles = params.num_vehicles;
    
    estimated_states = zeros(num_iterations, num_vehicles, 4);
    particle_trajectories = zeros(num_iterations, num_particles, num_vehicles, 4);
    weights = zeros(num_iterations, num_particles);
    
    for t = 1:num_iterations
        % Predict
        particles = PF.predict_particles(particles, params, ST);
        
        % Update weights
        weights(t, :) = PF.update_weights(particles, measurements(t), params, M);
        
        % Estimate state
        estimated_states(t, :, :) = PF.state_estimations(particles);
        
        % Store particle trajectories
        particle_trajectories(t, :, :, :) = particles;
        
        % Resample (systematic resampling)
        if t < num_iterations
            particles = systematic_resampling(particles, weights(t, :));
        end
    end
end

function resampled_particles = systematic_resampling(particles, weights)
    num_particles = size(particles, 1);
    positions = (rand + (0:num_particles-1)) / num_particles;
    cumulative_sum = cumsum(weights);
    i = 1;
    j = 1;
    resampled_particles = zeros(size(particles));
    
    while i <= num_particles
        if positions(i) < cumulative_sum(j)
            resampled_particles(i, :, :) = particles(j, :, :);
            i = i + 1;
        else
            j = j + 1;
        end
    end
end

function measurements = generate_measurements(ground_truth_states, params, M)
    num_iterations = size(ground_truth_states, 1);
    measurements = struct('c_tilde', [], 'o_tilde', [], 'v_avg_tilde', [], 'd_tilde', []);
    
    for t = 1:num_iterations
        d = ground_truth_states(t, :, 1);
        v = ground_truth_states(t, :, 2);
        
        measurements.c_tilde(t) = M.measure_c(M.count_loop(params, d, v));
        measurements.o_tilde(t) = M.measure_o(M.presence_loop(params, d, v), params);
        measurements.v_avg_tilde(t) = M.measure_v_avg(M.speed_loop(params, d, v));
        measurements.d_tilde(t, :) = M.measure_d(d, params);
    end
end
function plot_decision_making_validation(ground_truth_states, estimated_states, particle_trajectories, weights, params)
    figure('Position', [100, 100, 1200, 900]);
    state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};
    
    for i = 1:4
        subplot(2, 2, i);
        hold on;
        
        % Plot ground truth
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, 1, i)), 'g-', 'LineWidth', 2);
        
        % Plot particle trajectories
        for j = 1:params.num_particles
            plot(1:params.num_iterations, squeeze(particle_trajectories(:, j, 1, i)), 'Color', [0.7, 0.7, 0.7, 0.1], 'LineWidth', weights(end, j)*10);
        end
        
        title(state_names{i});
        xlabel('Time step');
        ylabel(state_names{i});
        legend('Ground Truth', 'Particles');
        hold off;
    end
    
    sgtitle('Decision Making Model Validation (Single Vehicle)');
end

function plot_particle_filter_validation(ground_truth_states, estimated_states, particle_trajectories, weights, params)
    figure('Position', [100, 100, 1200, 900]);
    state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};
    
    for i = 1:4
        subplot(2, 2, i);
        hold on;
        
        % Plot ground truth
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, 1, i)), 'g-', 'LineWidth', 2);
        
        % Plot particle trajectories
        for j = 1:params.num_particles
            plot(1:params.num_iterations, squeeze(particle_trajectories(:, j, 1, i)), 'Color', [0.7, 0.7, 0.7, 0.1], 'LineWidth', weights(end, j)*10);
        end
        
        % Plot estimated state
        plot(1:params.num_iterations, squeeze(estimated_states(:, 1, i)), 'b-', 'LineWidth', 2);
        
        title(state_names{i});
        xlabel('Time step');
        ylabel(state_names{i});
        legend('Ground Truth', 'Particles', 'Estimated');
        hold off;
    end
    
    % Plot loop placement in position subplot
    subplot(2, 2, 1);
    yline(params.d_loop1, 'r--', 'Vehicle Count Loop');
    
    sgtitle('Particle Filter Validation (Single Vehicle)');
end

function plot_multiple_vehicles(ground_truth_states, estimated_states, particle_trajectories, weights, params)
    figure('Position', [100, 100, 1200, 900]);
    state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};
    colors = [0, 0.5, 0; 0, 0.7, 0];  % Different shades of green
    
    for i = 1:4
        subplot(2, 2, i);
        hold on;
        
        for v = 1:params.num_vehicles
            % Plot ground truth
            plot(1:params.num_iterations, squeeze(ground_truth_states(:, v, i)), 'Color', colors(v, :), 'LineWidth', 2);
            
            % Plot particle trajectories
            for j = 1:params.num_particles
                plot(1:params.num_iterations, squeeze(particle_trajectories(:, j, v, i)), 'Color', [0.7, 0.7, 0.7, 0.1], 'LineWidth', weights(end, j)*10);
            end
            
            % Plot estimated state
            plot(1:params.num_iterations, squeeze(estimated_states(:, v, i)), 'b-', 'LineWidth', 2);
            
            % Calculate and plot standard deviation
            std_dev = std(squeeze(particle_trajectories(:, :, v, i)), 0, 2);
            errorbar(1:params.num_iterations, squeeze(estimated_states(:, v, i)), std_dev, 'r', 'LineStyle', 'none');
        end
        
        title(state_names{i});
        xlabel('Time step');
        ylabel(state_names{i});
        legend('Ground Truth 1', 'Ground Truth 2', 'Particles', 'Estimated 1', 'Estimated 2', 'Std Dev');
        hold off;
    end
    
    sgtitle('Multiple Vehicles Particle Filter Results');
end
function mae = calculate_mae(ground_truth_states, estimated_states)
    mae = mean(abs(ground_truth_states - estimated_states), 'all');
end

function save_results(ground_truth_states, estimated_states, particle_trajectories, weights, mae, params)
    % Create a folder to save the results
    current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
    folder_path = fullfile('results', char(current_time));
    mkdir(folder_path);
    
    % Save data
    save(fullfile(folder_path, 'results.mat'), 'ground_truth_states', 'estimated_states', 'particle_trajectories', 'weights', 'mae', 'params');
    
    % Save figures
    figures = findobj('Type', 'figure');
    for i = 1:length(figures)
        figure(figures(i));
        saveas(gcf, fullfile(folder_path, sprintf('figure_%d.fig', i)));
        saveas(gcf, fullfile(folder_path, sprintf('figure_%d.png', i)));
    end
    
    % Save MAE to a text file
    fid = fopen(fullfile(folder_path, 'mae.txt'), 'w');
    fprintf(fid, 'Mean Absolute Error: %f\n', mae);
    fclose(fid);
    
    disp(['Results saved in folder: ' folder_path]);
end
function plot_ground_truth(ground_truth_states, ground_truth_measurements, params)
   

    % Create a new figure
    figure('Position', [100, 100, 1200, 900]);

    % 1. Vehicle location and traffic signal status
    subplot(2, 2, 1);
    hold on;
    color_map = containers.Map({'red', 'yellow', 'green'}, {'r', 'y', 'g'});
    for t = 1:params.num_iterations
        color = color_map(params.traffic_signal_states{t});
        plot(t, params.d_stop_line, [color, '.'], 'MarkerSize', 10);
    end
    for i = 1:params.num_vehicles
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, i, 1)), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
    end
    yline(params.d_loop1, 'r--', 'LineWidth', 1);
    yline(params.d_loop2, 'b--', 'LineWidth', 1);
    yline(params.d_loop3, 'g--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Vehicle Positions and Traffic Signals');
    ylim([0, params.d_stop_line + 50]);
    legend('Location', 'best');
    grid on;

    % 2. Vehicle speed
    subplot(2, 2, 2);
    hold on;
    for i = 1:params.num_vehicles
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, i, 2)), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
    end
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Vehicle Velocities');
    legend('Location', 'best');
    grid on;

    % 3. Vehicle acceleration
    subplot(2, 2, 3);
    hold on;
    for i = 1:params.num_vehicles
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, i, 3)), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
    end
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    title('Vehicle Accelerations');
    legend('Location', 'best');
    grid on;

    % 4. Vehicle Decision
    subplot(2, 2, 4);
    hold on;
    for i = 1:params.num_vehicles
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, i, 4)), 'LineWidth', 2, 'DisplayName', sprintf('Vehicle %d', i));
    end
    xlabel('Time (s)');
    ylabel('Decision');
    title('Vehicle Decisions');
    yticks([1, 2, 3]);
    yticklabels({'Stop', 'Go', 'Undecided'});
    legend('Location', 'best');
    grid on;

    % Add a general title
    sgtitle('Ground Truth States', 'FontSize', 16);
end