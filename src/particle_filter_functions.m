function PF = particle_filter_functions()
    PF.initialize_particles = @initialize_particles;
    PF.predict_particles = @predict_particles;
    PF.update_weights = @update_weights;
    PF.logsumexp = @logsumexp;
    PF.resample_particles = @resample_particles;
    PF.state_estimations = @state_estimations;
    PF.queue_estimations = @queue_estimations;
end
%% initialize_particles
function particles = initialize_particles(init_bounds, params)
    num_particles = params.num_particles;
    num_vehicles = params.num_vehicles;
    
    % Initialize particles uniformly within the given bounds
    state_dim = size(init_bounds, 1);
    particles = zeros(num_particles, num_vehicles, state_dim);
    decision_values = params.D_undecided; % Initial decision is undecided
    
    % Minimum safe distance between vehicles
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

function particles = predict_particles(particles, params, ST)
    % Get the size of the particles array
    [num_particles, num_vehicles, state_dim] = size(particles);
    
    % Error checking
    if state_dim < 4
        error('Particle state dimension is less than 4. Expected at least 4 (d, v, a, D), got %d', state_dim);
    end
    
    % Generate traffic signal states for all time steps
    S = ST.generate_traffic_signal_states(params);
    
    % Initialize T_elapsed
    [~, T_elapsed_dt, ~, ~] = ST.update_elapsed_time(S, params);
    T_elapsed_dt = T_elapsed_dt(1, :);  % Only keep current time step

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
        current_state = squeeze(particles(i, :, :))';  % Transpose to make it [num_vehicles x state_dim]
        next_state = ST.nextState(current_state, params);
        d_next(i, :) = next_state(:, 1)';
        v_next(i, :) = next_state(:, 2)';
        
        % Calculate a_IDM_next, D_next, a_decision_next, and a_next for this particle
        a_IDM_next = ST.intelligent_driver_model(d(i, :), v(i, :), params);
        [~, T_elapsed_next_dt] = ST.calculate_T_elapsed_next(T_elapsed_dt, params);
        D_next(i, :) = ST.decision_making(d(i, :), d_next(i, :), v(i, :), v_next(i, :), S{1}, S{2}, T_elapsed_dt, T_elapsed_next_dt, D(i, :), params);
        a_decision_next = ST.traffic_light_decision_model(D(i, :), v(i, :), d(i, :), params);
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
    num_particles = size(particles, 1);
    num_vehicles = params.num_vehicles;
    log_weights = zeros(num_particles, num_vehicles);

    for i = 1:num_vehicles
        d = particles(:, i, 1);
        v = particles(:, i, 2);

        c = M.count_loop(params, d, v);
        o = M.presence_loop(params, d, v);
        v_avg = M.speed_loop(params, d, v);

        log_weights(:, i) = 0;  % Initialize log weights

        if isfield(measurement, 'c_tilde')
            log_weights(:, i) = log_weights(:, i) + log(M.count_loop_probability_density(measurement.c_tilde, c));
        end
        if isfield(measurement, 'o_tilde')
            log_weights(:, i) = log_weights(:, i) + log(M.presence_loop_probability(measurement.o_tilde, o, params));
        end
        if isfield(measurement, 'v_avg_tilde')
            log_weights(:, i) = log_weights(:, i) + log(M.speed_loop_probability(measurement.v_avg_tilde, v_avg));
        end
        if isfield(measurement, 'd_tilde')
            log_weights(:, i) = log_weights(:, i) + log(M.GPS_probability(measurement.d_tilde(i), d, params));
        end
    end

    % Normalize log weights
    log_weights = log_weights - logsumexp(log_weights, 1);
end
%% logsumexp

function s = logsumexp(x, dim)
    % Compute log(sum(exp(x), dim)) while avoiding numerical underflow.
    % By default dim = 1 (columns).
    if nargin == 1
        % Determine which dimension sum will use
        dim = find(size(x)~=1, 1);
        if isempty(dim), dim = 1; end
    end

    % Subtract the largest in each column
    y = max(x, [], dim);
    x = bsxfun(@minus, x, y);
    s = y + log(sum(exp(x), dim));
    i = find(~isfinite(y));
    if ~isempty(i)
        s(i) = y(i);
    end
end
%% resampled_particles

function resampled_particles = resample_particles(particles, log_weights)
    [num_particles, num_vehicles, state_dim] = size(particles);
    
    weights = exp(log_weights - max(log_weights, [], 1));  % Convert back to linear scale
    weights = weights ./ sum(weights, 1);
    
    resampled_particles = zeros(num_particles, num_vehicles, state_dim);
    
    for v = 1:num_vehicles
        cumulative_sum = cumsum(weights(:, v));
        u = (0:num_particles-1)' / num_particles + rand() / num_particles;
        [~, indices] = histc(u, [0; cumulative_sum]);
        resampled_particles(:, v, :) = particles(indices, v, :);
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