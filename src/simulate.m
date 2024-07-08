function [state, S, total_cpu_times] = simulate(params, SHOW_TIMING_OUTPUT)

    % Check params
    required_fields = {'num_iterations', 'num_vehicles', 'd_stop_line', 'v_desired', 'D_undecided'};
    for i = 1:length(required_fields)
        if ~isfield(params, required_fields{i})
            error('Missing required parameter: %s', required_fields{i});
        end
    end
    
    % Print params for debugging
    disp('Params:');
    disp(params);


    % Initialize state matrix
    % state: [d, v, a, D, lane]
    state = zeros(params.num_iterations, params.num_vehicles, 5);
    state(:, :, 4) = params.D_undecided;
    state(:, :, 5) = params.lane;
    num_vehicles = params.num_vehicles;
    num_iterations = params.num_iterations;
    initial_offset = params.d_0; % meters

    % Initialize first state
    initial_spacing = max(params.s0, rand() * 10); % meters
    for i = 1:num_vehicles
        state(1, i, 1) = initial_offset + (params.num_vehicles - i) * initial_spacing; % Position
        state(1, i, 2) = params.v_desired; % Velocity
        state(1, i, 3) = 0; % Acceleration
    end

    % After initialization, sorting is not necessary if we initialized correctly
    % But we can keep it as a safety measure
    [~, order] = sort(state(1, :, 1), 'descend');
    state(1, :, :) = state(1, order, :);
    
     
    % Generate traffic signal states
    StateTransition = state_transition();
    S = StateTransition.generate_traffic_signal_states(params);

    % Initialize elapsed time matrix
    T_elapsed = StateTransition.update_elapsed_time(S, params);

    % Initialize total CPU time
    total_cpu_start = cputime;
    total_cpu_times = struct('vehicle_relations', 0, 'a_IDM_next', 0, 'a_decision_next', 0, ...
                             'a_next', 0, 'next_state', 0, 'T_elapsed_next', 0, ...
                             'd_boundary', 0, 'D_next', 0, 'update_state', 0);

    % Main simulation loop
    for k = 1:num_iterations-1
        if SHOW_TIMING_OUTPUT
            fprintf('Iteration %d:\n', k);
        end

        % Calculate vehicle relations
        cpu_start = cputime;
        try
            fprintf('Debug: state(k,:,1) = %s\n', mat2str(squeeze(state(k,:,1))));
            fprintf('Debug: state(k,:,2) = %s\n', mat2str(squeeze(state(k,:,2))));
            [delta_v, s] = StateTransition.calculate_vehicle_relations(squeeze(state(k,:,1)), squeeze(state(k,:,2)), params);
        catch e
            fprintf('Error in calculate_vehicle_relations at iteration %d:\n', k);
            rethrow(e);
        end
        total_cpu_times.vehicle_relations = total_cpu_times.vehicle_relations + (cputime - cpu_start);

        % Debug information
        fprintf('Debug: delta_v = %s\n', mat2str(delta_v));
        fprintf('Debug: s = %s\n', mat2str(s));

          % Check for invalid delta_v or s
        if any(isnan(delta_v)) || any(isinf(delta_v))
            error('Invalid delta_v at iteration %d', k);
        end
        if any(isnan(s)) || (any(isinf(s(2:end))))
            error('Invalid s at iteration %d', k);
        end

        % Initialize next_state for all vehicles
        next_state = zeros(num_vehicles, 5);
        a_next = zeros(1, num_vehicles);

        % Calculate next states for all vehicles
        for i = 1:num_vehicles
            % Calculate a_IDM_next
            cpu_start = cputime;
            a_IDM_next = StateTransition.intelligent_driver_model(state(k,i,2), delta_v(i), s(i), params);
            total_cpu_times.a_IDM_next = total_cpu_times.a_IDM_next + (cputime - cpu_start);

            % Calculate a_decision_next
            cpu_start = cputime;
            a_decision_next = StateTransition.traffic_light_decision_model(state(k,i,4), state(k,i,2), state(k,i,1), params);
            total_cpu_times.a_decision_next = total_cpu_times.a_decision_next + (cputime - cpu_start);

            % Extract current vehicle speed
            v = state(k,i,2);

            % Calculate a_next
            cpu_start = cputime;
            fprintf('Value of v: %f\n', v);
            a_next(i) = StateTransition.acceleration_next(a_IDM_next, a_decision_next, params, v);
            total_cpu_times.a_next = total_cpu_times.a_next + (cputime - cpu_start);

            % Calculate next state
            cpu_start = cputime;
            next_state(i,:) = StateTransition.nextState(state(k,i,:), params);
            next_state(i,3) = a_next(i);  % Update acceleration
            total_cpu_times.next_state = total_cpu_times.next_state + (cputime - cpu_start);

            % Calculate tentative next state
            tentative_next_state = StateTransition.nextState(state(k,i,:), params);
            tentative_next_state(3) = a_next(i);  % Update acceleration
            
            % Store tentative next state
            next_state(i,:) = tentative_next_state;
        end

        % Check for invalid next_state
        if any(isnan(next_state(:))) || any(isinf(next_state(:)))
            error('Invalid next_state at iteration %d', k);
        end

        % Apply constraints
        min_distance = params.s0;

        for i = num_vehicles:-1:2
            % Check and correct overtaking situations
            if next_state(i, 1) > next_state(i-1, 1)
                % If overtaking occurs, adjust the position of the rear vehicle to the rear of the front vehicle                state(k+1, i, 1) = state(k+1, i-1, 1) - min_distance;
                next_state(i, 1) = next_state(i-1, 1) - min_distance;
                warning('Prevented overtaking at time step %d between vehicles %d and %d', k+1, i-1, i);
            end

            % Check and correct minimum safety distance
            distance = next_state(i-1, 1) - next_state(i, 1);
            if distance < min_distance
                % If the distance is less than the minimum safe distance, adjust the position of the rear vehicle
                next_state(i, 1) = next_state(i-1, 1) - min_distance;
                warning('Adjusted position to maintain minimum distance at time step %d between vehicles %d and %d', k+1, i-1, i);
            end
            % Update velocity to reflect position adjustments
            next_state(i, 2) = (next_state(i, 1) - state(k, i, 1)) / params.dt;
            
            % Update acceleration 
            next_state(i, 3) = (next_state(i, 2) - state(k, i, 2)) / params.dt;
        end

         % Update state matrix with constrained next_state
        state(k+1,:,:) = next_state;
        

        % Calculate T_elapsed_next
        cpu_start = cputime;
        T_elapsed_next = StateTransition.calculate_T_elapsed_next(T_elapsed(k,:), S(k), S(k+1), params);
        total_cpu_times.T_elapsed_next = total_cpu_times.T_elapsed_next + (cputime - cpu_start);

        % Calculate d_boundary
        cpu_start = cputime;
        d_b_next = params.d_stop_line - T_elapsed_next .* next_state(:,2)';
        d_a_next = params.T_reaction * next_state(:,2)' + (next_state(:,2)'.^2) / (2 * params.b_max);
        total_cpu_times.d_boundary = total_cpu_times.d_boundary + (cputime - cpu_start);

        % Update decision for all vehicles
        cpu_start = cputime;
        D_next = StateTransition.decision_making(state(k,:,1), next_state(:,1)', state(k,:,2), next_state(:,2)', ...
                                                 S(k), S(k+1), T_elapsed(k,:), T_elapsed_next, state(k,:,4), params);
        total_cpu_times.D_next = total_cpu_times.D_next + (cputime - cpu_start);


        % Update T_elapsed
        T_elapsed(k+1,:) = T_elapsed_next;

        % Check for invalid states
        check_invalid_states(state, k+1);

        % Check for invalid T_elapsed
        if any(isnan(T_elapsed(k+1,:))) || any(isinf(T_elapsed(k+1,:)))
            error('Invalid T_elapsed at iteration %d', k+1);
        end

         % Final safety check 
        for i = 2:num_vehicles
            if state(k+1, i, 1) > state(k+1, i-1, 1)
                warning('Unexpected vehicle order violation at time step %d between vehicles %d and %d', k+1, i-1, i);
            end
            if (state(k+1, i-1, 1) - state(k+1, i, 1)) < min_distance
                warning('Unexpected minimum distance violation at time step %d between vehicles %d and %d', k+1, i-1, i);
            end
        end
    end

    total_cpu_time = cputime - total_cpu_start;

    % Print total CPU time
    print_cpu_times(total_cpu_time, total_cpu_times);
end


function check_invalid_states(state, k)
    current_state = squeeze(state(k,:,:));
    if any(isnan(current_state(:))) || any(isinf(current_state(:)))
        error('Invalid state detected at iteration %d', k);
    end
end

function print_cpu_times(total_cpu_time, total_cpu_times)
    fprintf('Total CPU time: %.4f s\n', total_cpu_time);
    fprintf('CPU times breakdown:\n');
    fields = fieldnames(total_cpu_times);
    for i = 1:length(fields)
        fprintf('  %s: %.4f s\n', fields{i}, total_cpu_times.(fields{i}));
    end
end
