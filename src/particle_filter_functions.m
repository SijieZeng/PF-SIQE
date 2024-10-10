function PF = particle_filter_functions()
    PF.initialize_particles = @initialize_particles;
    PF.predict_particles = @predict_particles;
    PF.update_weights = @update_weights;
    PF.logsumexp = @logsumexp;
    PF.resample_particles = @resample_particles;
    PF.state_estimations = @state_estimations;
    PF.queue_estimations = @queue_estimations;
    PF.run_particle_filter = @run_particle_filter;
end
%% initialize_particles
function particles = initialize_particles(init_bounds, params)
    num_particles = params.num_particles;
    num_vehicles = params.num_vehicles;
    state_dim = size(init_bounds, 1);
    particles = zeros(num_particles, num_vehicles, state_dim);
    decision_values = params.D_undecided; % Initial decision is undecided
    min_safe_distance = params.s0 + params.vehicle_length;
    
    % Validate init_bounds
    if any(init_bounds(:,1) >= init_bounds(:,2))
        error('Invalid init_bounds: min values must be less than max values');
    end
    
    for j = 1:num_particles
        % Initialize vehicles
        for i = 1:num_vehicles
            if i == 1
                % Initialize leading vehicle (i=1)
                particles(j, i, 1) = init_bounds(1, 2) - (init_bounds(1, 2) - init_bounds(1, 1)) * rand(); % Position (start from the back)
            else
                % Initialize following vehicles
                max_pos = particles(j, i-1, 1) - min_safe_distance;
                min_pos = max(init_bounds(1, 1), particles(j, i-1, 1) - (num_vehicles-i+1)*min_safe_distance);
                if max_pos > min_pos
                    particles(j, i, 1) = min_pos + (max_pos - min_pos) * rand();
                else
                    particles(j, i, 1) = max_pos - rand() * min_safe_distance/2; % Add some randomness
                end
            end
            
            % Initialize velocity
            particles(j, i, 2) = init_bounds(2, 1) + (init_bounds(2, 2) - init_bounds(2, 1)) * rand();
            
            % Initialize acceleration
            particles(j, i, 3) = init_bounds(3, 1) + (init_bounds(3, 2) - init_bounds(3, 1)) * rand();
            
            % Initialize decision
            particles(j, i, 4) = decision_values();
        end
        
        % Ensure the constraint is satisfied
        while any(diff(particles(j, :, 1)) > -min_safe_distance)
            % If constraint is violated, reinitialize positions
            for i = 2:num_vehicles
                max_pos = particles(j, i-1, 1) - min_safe_distance;
                min_pos = max(init_bounds(1, 1), particles(j, i-1, 1) - (num_vehicles-i+1)*min_safe_distance);
                if max_pos > min_pos
                    particles(j, i, 1) = min_pos + (max_pos - min_pos) * rand();
                else
                    particles(j, i, 1) = max_pos - rand() * min_safe_distance/2;
                end
            end
        end
    end
    
    % Validation
    for j = 1:num_particles
        for i = 1:num_vehicles
            if particles(j, i, 3) < init_bounds(3, 1) || particles(j, i, 3) > init_bounds(3, 2)
                error('Particle acceleration out of bounds for particle %d, vehicle %d', j, i);
            end
            if i > 1 && particles(j, i-1, 1) - particles(j, i, 1) < min_safe_distance
                error('Minimum safe distance violated for particle %d, between vehicles %d and %d', j, i-1, i);
            end
        end
    end
end
%% predict_particles

function particles = predict_particles(particles, params, current_signal, next_signal)
    num_particles = params.num_particles;
    num_vehicles = params.num_vehicles;

    % Assert expected state dimension
    assert(size(particles, 3) >= 4, 'Particle state dimension is less than 4. Expected at least 4 (position, velocity, acceleration, decision)');

    % Initialize T_elapsed
    ST = state_transition_functions();
    [T_elapsed_dt, ~] = ST.update_elapsed_time({current_signal}, params);
    T_elapsed_dt = T_elapsed_dt(1, :); % Only keep current time step

    % Vectorize operations
    d = particles(:, :, 1);
    v = particles(:, :, 2);
    D = particles(:, :, 4);

    % Calculate next states for each particle separately
    d_next = zeros(num_particles, num_vehicles);
    v_next = zeros(num_particles, num_vehicles);
    a_next = zeros(num_particles, num_vehicles);
    D_next = zeros(num_particles, num_vehicles);

    for i = 1:num_particles
        current_state = squeeze(particles(i, :, :))'; % Transpose to make it [num_vehicles x state_dim]
        next_state = ST.nextState(current_state, params);
        d_next(i, :) = next_state(:, 1)';
        v_next(i, :) = next_state(:, 2)';

        % Calculate a_IDM_next, D_next, a_decision_next, and a_next for this particle
        a_IDM_next = ST.intelligent_driver_model(d(i, :), v(i, :), params);
        T_elapsed_next_dt = ST.calculate_T_elapsed_next(T_elapsed_dt);
        D_next(i, :) = ST.decision_making(d(i, :), d_next(i, :), v(i, :), v_next(i, :), current_signal, next_signal, T_elapsed_dt, T_elapsed_next_dt, D(i, :), params);
        a_decision_next = ST.traffic_light_decision_model(D(i, :), v(i, :), d(i, :), {current_signal, next_signal}, params);
        a_next(i, :) = ST.acceleration_next(a_IDM_next, a_decision_next, v(i, :), params);
    end

    % Update particles
    particles(:, :, 1) = d_next;
    particles(:, :, 2) = v_next;
    particles(:, :, 3) = a_next;
    particles(:, :, 4) = D_next;
end

%% update_weights

function log_weights = update_weights(particles, measurement, params, M)
    [num_particles, num_vehicles, ~] = size(particles);
    log_weights = zeros(num_particles, num_vehicles);

    
        d = particles(:, :, 1);
        v = particles(:, :, 2);

        c = M.count_loop(params, d, v);
        o = M.presence_loop(params, d, v);
        v_avg = M.speed_loop(params, d, v);

        % Update weights for each measurement type
    measurement_types = {'c_tilde', 'o_tilde', 'v_avg_tilde', 'd_tilde'};
    for meas_type = measurement_types
        if isfield(measurement, meas_type{1})
            switch meas_type{1}
                case 'c_tilde'
                    log_weights = log_weights + log(M.count_loop_probability_density(measurement.c_tilde, c));
                case 'o_tilde'
                    log_weights = log_weights + log(M.presence_loop_probability(measurement.o_tilde, o, params));
                case 'v_avg_tilde'
                    log_weights = log_weights + log(M.speed_loop_probability(measurement.v_avg_tilde, v_avg));
                case 'd_tilde'
                    for i = 1:num_vehicles
                        log_weights(:, i) = log_weights(:, i) + log(M.GPS_probability(measurement.d_tilde(i), d(:, i), params));
                    end
            end
        end
    end

    % Normalize log weights
    log_weights = log_weights - logsumexp(log_weights, 1);
end

%% 
% Helper function for log-sum-exp trick
function s = logsumexp(x, dim)
    max_x = max(x, [], dim);
    exp_x = exp(x - max_x);
    s = log(sum(exp_x, dim)) + max_x;
end

%% resampled_particles

function resampled_particles = resample_particles(particles, log_weights, params)
ESS_threshold = params.ESS_threshold;
    [num_particles, num_vehicles, state_dim] = size(particles);
    
    weights = exp(log_weights - max(log_weights, [], 1));  % Convert back to linear scale
    weights = weights ./ sum(weights, 1);
    
    % Calculate Effective Sample Size (ESS)
    ESS = 1 ./ sum(weights.^2, 1);
    
    resampled_particles = particles;
    
    for v = 1:num_vehicles
        if ESS(v) < ESS_threshold * num_particles
            cumulative_sum = cumsum(weights(:, v));
            u = ((0:num_particles-1)' + rand()) / num_particles;
            [~, indices] = histc(u, [0; cumulative_sum]);
            resampled_particles(:, v, :) = particles(indices, v, :);
            
            % Add small random perturbation to maintain diversity
            resampled_particles(:, v, :) = resampled_particles(:, v, :) + ...
                randn(num_particles, 1, state_dim) * 1e-3;
        end
    end
end
%% estimated_state

function estimated_state = state_estimations(particles, weights)
    % Estimate the state by calculating the weighted average of particles
    normalized_weights = weights / sum(weights);
    estimated_state = sum(bsxfun(@times, particles, normalized_weights'), 1);
end
%% 

function queue_length = queue_estimations(estimated_state, params)
    % Estimate the queue length based on the estimated state
    queue_length = 0;
    for v = 1:params.num_vehicles
        if estimated_state(v, 1) < params.d_stop_line
            queue_length = queue_length + 1;
        end
    end
end
%% 
function [estimated_states, queue_lengths] = run_particle_filter(measurements, params, PF, ST, M)
    % Initialize particles
    init_bounds = [params.init_pos_min, params.init_pos_max;
                   params.init_vel_min, params.init_vel_max;
                   params.init_acc_min, params.init_acc_max];
    particles = PF.initialize_particles(init_bounds, params);

    % Initialize weights
    log_weights = zeros(params.num_particles, params.num_vehicles);

    % Initialize output arrays
    %                                        
    num_timesteps = length(measurements);
    estimated_states = zeros(num_timesteps, params.num_vehicles, 4);
    queue_lengths = zeros(num_timesteps, 1);

    for t = 1:num_timesteps
        % Predict step
        particles = PF.predict_particles(particles, params, ST);

        % Update step
        log_weights = PF.update_weights(particles, measurements{t}, params, M);

        % Estimate state
        weights = exp(log_weights - PF.logsumexp(log_weights, 1));
        estimated_state = PF.state_estimations(particles, weights);
        estimated_states(t, :, :) = estimated_state;

        % Estimate queue length
        queue_lengths(t) = PF.queue_estimations(estimated_state, params);

        % Resample step
        particles = PF.resample_particles(particles, log_weights);

        % Reset weights after resampling
        log_weights = zeros(params.num_particles, params.num_vehicles);
    end
end