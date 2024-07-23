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

function particles = predict_particles(particles, params, ST)
    % Get the size of the particles array
    [num_particles, num_vehicles, state_dim] = size(particles);
    
    % Error checking
    if state_dim < 4
        error('Particle state dimension is less than 4. Expected at least 4 (d, v, a, D), got %d', state_dim);
    end
    
    % Generate traffic signal states for all time steps (assuming this is done once)
    S = ST.generate_traffic_signal_states(params);
    
    % Initialize T_elapsed (assuming this is done once)
    [~, T_elapsed_dt, ~, ~] = ST.update_elapsed_time(S, params);
    
    for j = 1:num_particles
        % Get states of all vehicles for this particle
        if num_vehicles == 1
            all_vehicle_states = squeeze(particles(j, 1, :))';
        else
            all_vehicle_states = squeeze(particles(j, :, :));
        end
        
        % If there's only one vehicle, ensure all_vehicle_states is a column vector
        %if num_vehicles == 1
            %all_vehicle_states = all_vehicle_states(:);
        %end
        
        % Extract d, v, and D from all_vehicle_states
        d = all_vehicle_states(:, 1)';
        v = all_vehicle_states(:, 2)';
        a = all_vehicle_states(:, 3)';
        D = all_vehicle_states(:, 4)';
        
        % Convert all_vehicle_states to matrix format
        states_matrix = [all_vehicle_states(:, 1:4), ones(num_vehicles, 1)];
        
        % Call nextState with the matrix format
        next_states_matrix = ST.nextState(states_matrix, params);
        
        % Extract d_next and v_next
        d_next = next_states_matrix(:, 1)';
        v_next = next_states_matrix(:, 2)';
        
        % Calculate a_IDM_next
        a_IDM_next = ST.intelligent_driver_model(d, v, params);
        
        % Calculate T_elapsed_next
        [~, T_elapsed_next_dt] = ST.calculate_T_elapsed_next(T_elapsed_dt, params);
        
        % Ensure T_elapsed_next_dt length is correct
        if length(T_elapsed_next_dt) ~= num_vehicles
            T_elapsed_next_dt = zeros(1, num_vehicles);
        end
        
        % Calculate D_next (assuming we're at step 1 for simplicity)
        D_next = ST.decision_making(d, d_next, v, v_next, S{1}, S{2}, T_elapsed_dt, T_elapsed_next_dt, D, params);
        
        % Calculate a_decision_next
        a_decision_next = ST.traffic_light_decision_model(D, v, d, params);
        
        % Calculate a_next
        a_next = ST.acceleration_next(a_IDM_next, a_decision_next, v, params);
        
        % Update particles with new states
        for i = 1:num_vehicles
            particles(j, i, :) = [d_next(i), v_next(i), a_next(i), D_next(i)];
        end
    end
end
%% update_weights

function weights = update_weights(particles, measurements, params, M)
    num_particles = size(particles, 1);
    num_vehicles = params.num_vehicles;
    weights = zeros(num_particles, num_vehicles);

    for i = 1:num_vehicles
        for j = 1:num_particles
            particle_state = squeeze(particles(j, :, i));

            % Count loop
            c = M.count_loop(params, particle_state(1), particle_state(2));
            c_tilde = measurements.c_tilde(i);
            p_c = M.count_loop_probability_density(c_tilde, c);

            % Presence loop
            o = M.presence_loop(params, particle_state(1), particle_state(2));
            o_tilde = measurements.o_tilde(i);
            p_o = M.presence_loop_probability(o_tilde, o, params);

            % Speed loop
            v_avg = M.speed_loop(params, particle_state(1), particle_state(2));
            v_avg_tilde = measurements.v_avg_tilde(i);
            p_v = M.speed_loop_probability(v_avg_tilde, v_avg);

            % GPS measurement
            d = particle_state(1);
            d_tilde = measurements.d_tilde(i);
            p_G = M.GPS_probability(d_tilde, d, params);

            % Joint probability
            weights(j, i) = M.measurement_probability(c_tilde, c, o_tilde, o, v_avg_tilde, v_avg, d_tilde, d, params);
        end
    end

    % Normalize weights
    weights = weights ./ sum(weights, 1);
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