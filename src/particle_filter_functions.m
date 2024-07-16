function PF = particle_filter_functions()
    PF.initialize_particles = @initialize_particles;
    PF.predict_particles = @predict_particles;
    PF.update_weights = @update_weights;
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

function particles = predict_particles(particles, params, StateTransition)
    [num_particles, num_vehicles, state_dim] = size(particles);

    for j = 1:num_particles
        % Get states of all vehicles for this particle
        all_vehicle_states = squeeze(particles(j, :, :));
        
        for i = 1:num_vehicles
            % Predict next state
            current_state = all_vehicle_states(i, :)';
            try
                next_state = StateTransition.nextState(current_state, all_vehicle_states, params);
                
                % Validate next_state
                if length(next_state) ~= state_dim
                    error('Invalid next state: dimension mismatch. Expected %d, got %d', state_dim, length(next_state));
                end
                if any(isnan(next_state)) || any(isinf(next_state))
                    error('Invalid next state: contains NaN or Inf');
                end
                if next_state(1) < 0  % Assuming first state is position
                    error('Invalid next state: negative position');
                end
                
                particles(j, i, :) = next_state;
            catch ME
                warning('Error in state prediction for particle %d, vehicle %d: %s', j, i, ME.message);
                % Keep the current state if prediction fails
            end
        end
    end
end
%% update_weights

function weights = update_weights(particles, measurements, params, Measurement)
    num_particles = size(particles, 1);
    num_vehicles = params.num_vehicles;
    
    weights = zeros(num_particles, 1);
    for p = 1:num_particles
        particle_state = squeeze(particles(p, :, :));
        
        % Count loop
        c_actual = Measurement.count_loop(params, particle_state(:, 1), particle_state(:, 2));
        c_measured = measurements.c_tilde;
        p_c = Measurement.count_loop_probability(c_measured, c_actual);
        
        % Presence loop
        o_actual = Measurement.presence_loop(params, particle_state(:, 1), particle_state(:, 2));
        o_measured = measurements.o_tilde;
        p_o = Measurement.presence_loop_probability(o_measured, o_actual, params);
        
        % Speed loop
        v_avg_actual = Measurement.speed_loop(params, particle_state(:, 1), particle_state(:, 2));
        v_avg_measured = measurements.v_avg_tilde;
        p_v = Measurement.speed_loop_probability(v_avg_measured, v_avg_actual);
        
        % GPS measurement
        d_actual = particle_state(:, 1);
        d_measured = measurements.d_tilde;
        p_d = Measurement.GPS_probability(d_measured, d_actual, params);
        
        % Joint probability
        weights(p) = p_c * p_o * p_v * p_d;
    end
    
    % Normalize weights
    weights = weights / sum(weights);
end
%% resampled_particles

function resampled_particles = resample_particles(particles, weights)
    num_particles = size(particles, 1);
    num_vehicles = size(particles, 2);
    state_dim = size(particles, 3);
    
    cumulative_sum = cumsum(weights);
    resampled_particles = zeros(num_particles, num_vehicles, state_dim);
    
    for p = 1:num_particles
        random_value = rand();
        index = find(cumulative_sum >= random_value, 1);
        resampled_particles(p, :, :) = particles(index, :, :);
    end
end
%% estimated_state

function estimated_state = state_estimations(particles)
    % Estimate the state by averaging over all particles
    estimated_state = mean(particles, 1);
end

function queue_length = queue_estimations(estimated_state, params)
    % Estimate the queue length based on the estimated state
    queue_length = 0;
    for v = 1:params.num_vehicles
        if estimated_state(v, 1) < params.d_stop_line
            queue_length = queue_length + 1;
        end
    end
end