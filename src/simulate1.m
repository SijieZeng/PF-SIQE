function [all_states, all_signals] = simulate1(initial_states, params)
    % Initialize
    ST = state_transition();
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;
    
    % Preallocate arrays to store states and signals
    all_states = cell(num_iterations, 1);
    all_signals = cell(num_iterations, 1);

    % Initialize first state
    current_states = initial_states;
    all_states{1} = current_states;
    


    % Generate traffic signal states for all time steps
    S = ST.generate_traffic_signal_states(params);
    for i = 1:length(S)
    all_signals{i} = S{i};
    end
    
    % Initialize T_elapsed
    [~, T_elapsed_dt, ~, ~] = ST.update_elapsed_time(S, params);

    for step = 1:num_iterations-1
        % 在调用 nextState 之前，从 current_states 中提取 d 和 v
        d = [current_states.d];
        v = [current_states.v];
        D = [current_states.D];

        % Convert current_states to matrix format
        states_matrix = zeros(length(current_states), 5);
        for i = 1:length(current_states)
            states_matrix(i, 1) = current_states(i).d;
            states_matrix(i, 2) = current_states(i).v;
            states_matrix(i, 3) = current_states(i).a;
            states_matrix(i, 4) = current_states(i).D;
            % Assuming lane information is not available, set it to 1 for all vehicles
            states_matrix(i, 5) = 1;
        end
        
        % Call nextState with the matrix format
        next_states_matrix = ST.nextState(states_matrix, params);
        
        % Convert next_states_matrix back to struct array format
        next_states = struct('d', num2cell(next_states_matrix(:, 1)), ...
                             'v', num2cell(next_states_matrix(:, 2)), ...
                             'a', num2cell(next_states_matrix(:, 3)), ...
                             'D', num2cell(next_states_matrix(:, 4)));
     
        % Calculate next states
        d_next = [next_states.d];
        v_next = [next_states.v];
        
        % 检查长度
        if length(d) ~= params.num_vehicles || length(v) ~= params.num_vehicles
            error('Length of d (%d) or v (%d) does not match params.num_vehicles (%d)', ...
                  length(d), length(v), params.num_vehicles);
        end

        % Calculate a_IDM_next
        a_IDM_next = ST.intelligent_driver_model(d, v, params);
        
        % Calculate T_elapsed_next
        [~, T_elapsed_next_dt] = ST.calculate_T_elapsed_next(T_elapsed_dt, params);

        % Ensure T_elapsed_next_dt length is correct
        if length(T_elapsed_next_dt) ~= num_vehicles
            T_elapsed_next_dt = zeros(1, num_vehicles); % 初始化为零，或根据需要调整
        end

        % Debugging outputs for array lengths and indices
        %fprintf('Step %d: Checking array lengths and indices...\n', step);
        %fprintf('num_vehicles: %d\n', num_vehicles);
        %fprintf('Length of T_elapsed_next_dt: %d\n', length(T_elapsed_next_dt));
        %fprintf('Length of d_next: %d\n', length(d_next));
        %fprintf('Length of v_next: %d\n', length(v_next));
        %fprintf('Length of D: %d\n', length(D));
        % Calculate D_next
        D_next = ST.decision_making(d, d_next, v, v_next, S{step}, S{step+1}, T_elapsed_dt, T_elapsed_next_dt, D, params);
        
        % Calculate a_decision_next
        a_decision_next = ST.traffic_light_decision_model(D, v, d, params);
        
        % Calculate a_next
        a_next = ST.acceleration_next(a_IDM_next, a_decision_next, v, params);
        
        % Update states for next iteration
        for i = 1:num_vehicles
            next_states(i).a = a_next(i);
            next_states(i).D = D_next(i);
        end
        
        % Store states
        all_states{step+1} = next_states;

        % Print current state for debugging
        fprintf('Time step %d: S = %s, S_next = %s\n', step, S{step}, S{step+1});
        for i = 1:num_vehicles
            if i == 1
                s = Inf;
                delta_v = 0;
            else
                s = max(0, d(i-1) - d(i) - params.vehicle_length);
                delta_v = v(i) - v(i-1);
            end
            fprintf('Vehicle %d, d = %.2f, v = %.2f, a = %.2f, D = %d, delta_v = %.2f, s = %.2f, d_next = %.2f, v_next = %.2f\n', ...
                i, d(i), v(i), current_states(i).a, D(i), delta_v, s, d_next(i), v_next(i));
        end
        fprintf('\n');

        % Update for next iteration
        current_states = next_states;
        T_elapsed_dt = T_elapsed_next_dt;
    end
end