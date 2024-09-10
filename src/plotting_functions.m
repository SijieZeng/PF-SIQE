function PLOT = plotting_functions()
    PLOT.run_particle_filter = @run_particle_filter;
    PLOT.systematic_resampling = @systematic_resampling;
    PLOT.calculate_mae = @calculate_mae;
    PLOT.plot_decision_making_validation = @plot_decision_making_validation;
    PLOT.generate_ground_truth = @generate_ground_truth;
    PLOT.generate_measurements = @generate_measurements;
end
%%
function [estimated_states, particle_trajectories, weights] = run_particle_filter(particles, ground_truth_measurements, all_signals, params, PF, ST, M)
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
        % 创建当前时间步的测量结构
        current_measurement = struct();
        if isfield(ground_truth_measurements, 'c_tilde')
            current_measurement.c_tilde = ground_truth_measurements.c_tilde(t);
        end
        if isfield(ground_truth_measurements, 'o_tilde')
            current_measurement.o_tilde = ground_truth_measurements.o_tilde(t);
        end
        if isfield(ground_truth_measurements, 'v_avg_tilde')
            current_measurement.v_avg_tilde = ground_truth_measurements.v_avg_tilde(t);
        end
        if isfield(ground_truth_measurements, 'd_tilde')
            current_measurement.d_tilde = ground_truth_measurements.d_tilde(t, :);
        end
        
        weights(t, :) = PF.update_weights(particles, current_measurement, params, M);
        

        % Estimate state
        estimated_states(t, :, :) = reshape(PF.state_estimations(particles, weights(t, :)), [1, num_vehicles, size(particles, 3)]);
        % estimated_states(t, :, :) = PF.state_estimations(particles, weights(t, :));
        
        % Store particle trajectories
        particle_trajectories(t, :, :, :) = particles;
        
        % Resample (systematic resampling)
        if t < num_iterations
            particles = systematic_resampling(particles, weights(t, :));
        end
    end
end
%% 

function resampled_particles = systematic_resampling(particles, weights)
    num_particles = size(particles, 1);
    
    % 确保权重是归一化的
    weights = weights / sum(weights);
    
    positions = (rand + (0:num_particles-1)) / num_particles;
    cumulative_sum = cumsum(weights);
    
    % 确保最后一个累积和等于1
    cumulative_sum(end) = 1;
    
    i = 1;
    j = 1;
    resampled_particles = zeros(size(particles));
    
    while i <= num_particles
        while j <= num_particles && positions(i) > cumulative_sum(j)
            j = j + 1;
        end
        
        if j <= num_particles
            resampled_particles(i, :, :) = particles(j, :, :);
        else
            % 如果 j 超出范围，使用最后一个粒子
            resampled_particles(i, :, :) = particles(end, :, :);
        end
        
        i = i + 1;
    end
end
%% 

function mae = calculate_mae(ground_truth_states, estimated_states)
    error = abs(estimated_states - ground_truth_states);
    mae = mean(error(:));
end
%% 

function plot_decision_making_validation(ground_truth_states, estimated_states, particle_trajectories, weights, all_signals, params)
    figure('Position', [100, 100, 1200, 900]);
    state_names = {'Position', 'Velocity', 'Acceleration', 'Decision'};
    
    for i = 1:4
        subplot(2, 2, i);
        hold on;
        
        % Plot ground truth
        plot(1:params.num_iterations, squeeze(ground_truth_states(:, 1, i)), 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
        
        % Plot particle trajectories
        for j = 1:params.num_particles
            plot(1:params.num_iterations, squeeze(particle_trajectories(:, j, 1, i)), 'Color', [0.7, 0.7, 0.7, 0.1], 'LineWidth', weights(end, j)*10);
        end
        
        % Plot estimated state
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
end
%% 
 function [ground_truth_states, all_signals] = generate_ground_truth(params, ST, PF)
    % Initialize particles
    init_bounds = [0, params.D_h;
                   params.v_desired - 1, params.v_desired;
                   -params.b_max, params.a_max;
                   1, 3];  % 添加决策状态的边界

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

    % Generate traffic signal states
    all_signals = ST.generate_traffic_signal_states(params);

    % Run simulation
    [all_states, ~] = simulate1(initial_states, params);

    % Convert all_states to ground_truth_states matrix
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;
    ground_truth_states = zeros(num_iterations, num_vehicles, 4);
    for t = 1:num_iterations
        for i = 1:num_vehicles
            ground_truth_states(t, i, :) = [all_states{t}(i).d, all_states{t}(i).v, all_states{t}(i).a, all_states{t}(i).D];
        end
    end
end
%% 

function [c_tilde, o_tilde, v_avg_tilde, d_tilde] = generate_measurements(params, ground_truth_states)
    % Initialize measurement functions
    M = measurement_functions();

    % Extract necessary parameters
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;

    % Initialize arrays for measurements
    c = zeros(1, num_iterations);
    c_tilde = zeros(1, num_iterations);
    o = zeros(1, num_iterations);
    o_tilde = zeros(1, num_iterations);
    v_avg = zeros(1, num_iterations);
    v_avg_tilde = zeros(1, num_iterations);
    d_tilde = zeros(num_vehicles, num_iterations);

    % Extract positions and velocities from ground_truth_states
    positions = squeeze(ground_truth_states(:, :, 1))';  % Transpose to get [num_vehicles, num_iterations]
    velocities = squeeze(ground_truth_states(:, :, 2))';

    for t = 1:num_iterations
        % Count vehicles passing the loop
        c(t) = M.count_loop(params, positions(:, t), velocities(:, t));
        % Measure the count with added noise
        c_tilde(t) = M.measure_c(c(t));

        % Presence loop
        o(t) = M.presence_loop(params, positions(:, t), velocities(:, t));
        o_tilde(t) = M.measure_o(o(t), params);

        % Speed loop
        v_avg(t) = M.speed_loop(params, positions(:, t), velocities(:, t));
        v_avg_tilde(t) = M.measure_v_avg(v_avg(t));

        % GPS
        for i = 1:num_vehicles
            d_tilde(i, t) = M.measure_d(positions(i, t), params);
        end
    end
end
    
