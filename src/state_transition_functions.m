function ST = state_transition_functions()
    ST.nextState = @nextState;
    ST.intelligent_driver_model = @intelligent_driver_model;
    ST.generate_traffic_signal_states = @generate_traffic_signal_states;
    ST.update_elapsed_time = @update_elapsed_time;
    ST.calculate_T_elapsed_next = @calculate_T_elapsed_next;
    ST.decision_making = @decision_making;
    ST.traffic_light_decision_model = @traffic_light_decision_model;
    ST.acceleration_next_no_noise = @acceleration_next_no_noise;
    ST.acceleration_next = @acceleration_next;
    ST.acceleration_probability = @acceleration_probability;
    ST.decision_probability = @decision_probability;
    ST.state_transition_probability = @state_transition_probability;
end
%% nextState

function next_states = nextState(states, params)
    % states: a num_vehicles x 5 matrix, each row represents a vehicle state [d, v, a, D, lane]
    % params: a structure containing simulation parameters
    % Returns: next_states, a matrix of the same dimension as the input, representing the next state of all vehicles
    num_vehicles = size(states, 1);
    next_states = zeros(size(states, 1), 4);
    T = params.dt;  % time step
    
    assert(size(states, 1) == params.num_vehicles, 'Mismatch between states and params.num_vehicles');

    for i = 1:num_vehicles
        d = states(i, 1);  % Vertical position (m)
        v = states(i, 2);  % speed (m/s)
        a = states(i, 3);  % Acceleration (m/s^2)
        D = states(i, 4);  % decisions
        % lane = states(i, 5);  % lane

        % Make sure acceleration and time step are valid
        if T <= 0 || isnan(T) || isnan(a) || isinf(T) || isinf(a)
            error('Invalid parameters: T: %f, a: %f', T, a);
        end

        % Update speed and position
        v_next = max(0, v + a * T);
        d_next = d + v * T + 0.5 * a * T^2;

        if isnan(d_next) || isnan(v_next) || isinf(d_next) || isinf(v_next)
            error('Invalid state detected. d: %f, v: %f', d_next, v_next);
        end

        next_states(i, :) = [d_next, v_next, a, D];
    end
     status = true; % or false if any issues were encountered
end 
%% intelligent_driver_model

function a_IDM_next = intelligent_driver_model(d, v, params)
  % Intelligent Driver Model (IDM) function
    % Inputs:
    %   v - current speed of the vehicle
    %   delta_v - speed difference to the leading vehicle
    %   s - spacing gap to the leading vehicle
    %   params - struct containing IDM parameters
    % Outputs:
    %   a_IDM_next - computed acceleration at time step k+1

    if ~isstruct(params)
        error('params must be a struct');
    end
    if ~all(isfield(params, {'num_vehicles', 's0', 'vehicle_length', 'v_desired', 'a_max', 'b', 'T', 'delta'}))
        error('params is missing required fields');
    end

    status = struct('success', true, 'message', '');

    num_vehicles = params.num_vehicles;
    dt = params.dt;
    s0 = params.s0;
    vehicle_length = params.vehicle_length;
    v_desired = params.v_desired;
    a_max = params.a_max;
    b = params.b;
    T = params.T; % safe time headway
    delta = params.delta;

    % Check input dimensions
    if length(v) ~= num_vehicles || length(d) ~= num_vehicles 
        error('Input vectors v, and d must have length equal to num_vehicles');
    end

    % Initialize output vector
    delta_v = zeros(1, num_vehicles);
    s = zeros(1, num_vehicles);
    a_IDM_next = zeros(1, num_vehicles);

    
    % Calculate vehicle relations and IDM for each vehicle
    for i = 1:num_vehicles
        if i == 1
            % Lead vehicle (special case)
            delta_v(1) = 0;  % No relative velocity for lead vehicle
            s(1) = Inf;  % Infinite space ahead for lead vehicle

            if v(i) < v_desired
                a_IDM = min(a_max, (v_desired - v(i)) / dt);  % Linear acceleration to desired speed
            else
                a_IDM = 0;  % The expected speed has been reached or exceeded, maintain the current speed
            end
        else
            delta_v(i) = v(i) - v(i-1);
             if any(isnan(delta_v))
                status.success = false;
                status.message = sprintf('NaN values detected in delta_v: %s', mat2str(delta_v));
                return;
            end
            s(i) = max(0,  d(i-1) - d(i) - vehicle_length);  % Ensure non-negative spacing
            if s(i) < s0
                warning('Vehicle %d: Spacing (%.2f) is less than minimum gap (%.2f)', i, s(i), s0);
            end

            % Desired gap
            s_star = s0 + v(i) * T + (v(i) * delta_v(i)) / (2 * sqrt(a_max * b));
    
            % Acceleration
            a_IDM = a_max * (1 - (v(i) / v_desired)^delta - (s_star / s(i))^2);
        end
        % 
        
        % Ensure acceleration is within reasonable bounds
        a_IDM_next(i) = max(min(a_IDM, a_max), -b);
        % fprintf('Debug: Vehicle %d, a_IDM = %.4f\n', i, a_IDM_next(i));
    end
    
    % Check for invalid values
    epsilon = 1e-10; % Small tolerance for floating-point comparisons
    if any(isnan(delta_v)) || any(isnan(s)) || any(isnan(a_IDM_next)) || ...
       any(isinf(delta_v(2:end))) || any(s(2:end) < -epsilon) || any(isinf(a_IDM_next))
        if any(isnan(delta_v))
            error('NaN values detected in delta_v: %s', mat2str(delta_v));
        elseif any(isnan(s))
            error('NaN values detected in s: %s', mat2str(s));
        elseif any(isnan(a_IDM_next))
            error('NaN values detected in a_IDM_next: %s', mat2str(a_IDM_next));
        elseif any(isinf(delta_v(2:end)))
            error('Inf values detected in delta_v (excluding lead vehicle): %s', mat2str(delta_v(2:end)));
        elseif any(s(2:end) < -epsilon)
            error('Negative spacing detected: %s', mat2str(s(2:end)));
        elseif any(isinf(a_IDM_next))
            error('Inf values detected in a_IDM_next: %s', mat2str(a_IDM_next));
        end
    end
end
%% generate_traffic_signal_states

function S = generate_traffic_signal_states(params)
    num_iterations = params.num_iterations * 2;
    red_time = params.red_time;   
    yellow_time = params.yellow_time; 
    green_time = params.green_time; 
    delta_t  = params.delta_t;

    S = strings(num_iterations, 1);
    cycle_length = red_time + yellow_time + green_time;

    for k = 1:num_iterations
        time = (k - 1) * delta_t;
        cycle_position = mod(time, cycle_length);

        if cycle_position < green_time
            S(k) = "green";
        elseif cycle_position < green_time + yellow_time
            S(k) = "yellow";
        else
            S(k) = "red";
        end
    end
end
%% update_elapsed_time

function [T_elapsed_delta_t, T_elapsed_dt, yellow_start_delta_t, yellow_start_dt] = update_elapsed_time(S, params)
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;
    yellow_time = params.yellow_time;
    delta_t = params.delta_t;
    dt = params.dt;
    steps_per_dt = dt / delta_t;
    T_elapsed_delta_t = zeros(num_iterations, num_vehicles);
    T_elapsed_dt = zeros(ceil(num_iterations/steps_per_dt), num_vehicles);
    
    % 预分配 yellow_start_delta_t
    yellow_start_delta_t = zeros(1, ceil(num_iterations / (yellow_time / delta_t)));
    yellow_count = 0;
    
    for k = 2:num_iterations
        if S(k) == "yellow" && S(k-1) ~= "yellow"
            yellow_count = yellow_count + 1;
            yellow_start_delta_t(yellow_count) = k;
        end
    end
    
    % 裁剪 yellow_start_delta_t 到实际大小
    yellow_start_delta_t = yellow_start_delta_t(1:yellow_count);

    % 计算T_elapsed_delta_t
    for k = 1:num_iterations
        if S(k) == "yellow"
            yellow_start = max(yellow_start_delta_t(yellow_start_delta_t <= k));
            elapsed = (k - yellow_start) * delta_t;
            remaining_time = yellow_time - elapsed;
            T_elapsed_delta_t(k, :) = max(round(remaining_time * 2) / 2, 0);
        else
            T_elapsed_delta_t(k, :) = 0;
        end
    end

    % 计算T_elapsed_dt
    for k = 1:steps_per_dt:num_iterations
        dt_index = ceil(k / steps_per_dt);
        T_elapsed_dt(dt_index, :) = T_elapsed_delta_t(k, :);
    end

    % 计算黄灯开始的dt时间
    yellow_start_dt = ceil(yellow_start_delta_t / steps_per_dt);
end
%% calculate_T_elapsed_next

function [T_elapsed_next_delta_t, T_elapsed_next_dt] = calculate_T_elapsed_next(T_elapsed, params)
    delta_t = params.delta_t;
    dt = params.dt;
    steps_per_dt = dt / delta_t;
    num_vehicles = size(T_elapsed, 2);
    num_iterations = size(T_elapsed, 1) - 1;

    % Calculate T_elapsed_next_delta_t directly
    T_elapsed_next_delta_t = T_elapsed(2:end, :);

    % Initialize and calculate T_elapsed_next_dt
    T_elapsed_next_dt = zeros(ceil(num_iterations / steps_per_dt), num_vehicles);
    for k = 1:ceil(num_iterations / steps_per_dt)
        T_elapsed_next_dt(k, :) = T_elapsed(min(k*steps_per_dt + 1, size(T_elapsed, 1)), :);
    end
end
%% decision_making

function D_next = decision_making(d, d_next, v, v_next, S, S_next, T_elapsed_dt, T_elapsed_next_dt, D, params)
% Decision-making function based on current state and signal state
    % Inputs:
    %   d - position at time k
    %   d_next - position at time k+1
    %   v - speed at time k
    %   v_next - speed at time k+1
    %   S - signal state at time k
    %   S_next - signal state at time k+1
    %   T_elapsed - elapsed yellow time at time k
    %   T_elapsed_next - elapsed yellow time at time k+1
    %   D - decision at time k
    %   num_vehicles - number of vehicles
    %   params - struct containing all parameters
    % Output:
    %   D_next - decision at time k+1

    % Extract parameters from the params struct
    num_vehicles = params.num_vehicles;
    D_h = params.D_h;
    T_reaction = params.T_reaction;
    b = params.b_max;
    T_yellow = params.yellow_time;
    d_stop_line = params.d_stop_line;
    p = params.p_stop;
    d_0 = params.d_0;

     % Initialize output
    D_next = repmat(params.D_undecided, 1, num_vehicles);

    for i = 1:num_vehicles
        % Step 1: Check if the vehicle is before the decision horizon
        if d_0 <= d_next(i) && d_next(i) <= D_h
            D_next(i) = params.D_undecided; % D_undecided
            continue;  % Move to the next vehicle
        end

        % Step 2: Check if the vehicle is between decision horizon and stop line
        if D_h < d_next(i) && d_next(i) <= d_stop_line
            if S_next == "red"
                D_next(i) = params.D_stop; % D_stop
            elseif S_next == "green"
                D_next(i) = params.D_go; % D_go
            elseif S_next == "yellow"
                d_a_next = T_reaction * v_next(i) + (v_next(i)^2) / (2 * b);
                d_b_next = (T_yellow - T_elapsed_next_dt(i)) * v_next(i);

                if d_next(i) <= d_b_next
                    D_next(i) = params.D_stop; % D_stop
                elseif d_a_next <= d_next(i) 
                    D_next(i) = params.D_go; % D_go
                else % d_b_next < d_next(i) && d_next(i) < d_a_next
                    if D(i) == params.D_undecided % D_undecided
                        r = rand();
                        if r < p
                            D_next(i) = params.D_stop; % D_stop
                        else
                            D_next(i) = params.D_go; % D_go
                        end
                    else
                        D_next(i) = D(i);
                    end
                end
            end
            continue;
        end

        % Step 3: If the vehicle is past the stop line
        if d_next(i) > d_stop_line
            D_next(i) = params.D_go;
            continue;  % Move to the next vehicle
        end

        % Step 4: Maintain previous decision if already made during yellow light
        if S == "yellow" && S_next == "yellow" && D(i) ~= params.D_undecided
            d_b = (T_yellow - T_elapsed_dt(i)) * v(i);
            d_a = T_reaction * v(i) + (v(i)^2) / (2 * b);
            d_b_next = (T_yellow - T_elapsed_next_dt(i)) * v_next(i);
            d_a_next = T_reaction * v_next(i) + (v_next(i)^2) / (2 * b);

            if d_b < d(i) && d(i) < d_a && d_b_next < d_next(i) && d_next(i) < d_a_next
                D_next(i) = D(i);
            end
        end
    end
end
%% traffic_light_decision_model

function a_decision_next = traffic_light_decision_model(D, v, d, params)
    % Input:
    % D: Current decision (D_stop, D_go, or D_undecided)
    % v: Current velocity
    % d: Current distance to the intersection
    % params: Struct containing all necessary parameters
    
    % Extract parameters
    T_discrete = params.T_discrete; % Discretization time interval (1 s)
    a_prime = params.a; % Comfortable acceleration (1.0 m/s^2)
    v_desired = params.v_desired; % Desired speed
    d_stop_line = params.d_stop_line;
    num_vehicles = params.num_vehicles;
    a_max = params.a_max; % Maximum acceleration
    b = params.b; % Maximum comfortable deceleration

    % Check input dimensions
    if length(D) ~= num_vehicles || length(v) ~= num_vehicles || length(d) ~= num_vehicles 
        error('Input vectors D, v, and d must have length equal to num_vehicles');
    end


    % Initialize output vector
    a_decision_next = zeros(1, num_vehicles);
     
% Determine a_decision_next based on the current decision and conditions
% for each vehicle
    for i = 1:num_vehicles
        if D(i) == params.D_stop
            if v(i) > 0
                C_s = max(1, 2 * (d_stop_line - d(i)) / (v(i) * T_discrete));
                a_decision_next(i) = -v(i) / (C_s * T_discrete);
            else
                a_decision_next(i) = 0; % Vehicle is already stopped
            end
        elseif D(i) == params.D_go || D(i) == params.D_undecided
            if v(i) > v_desired
                a_decision_next(i) = 0;
            else
                a_decision_next(i) = a_prime * (1 - (v(i) / v_desired)^2);
            end
        else
            error('Invalid decision state for vehicle %d', i);
        end
    
        % Ensure acceleration is within reasonable bounds
        a_decision_next(i) = max(min(a_decision_next(i), a_max), -b);
    end
end
%% acceleration_next_no_noise

function a_next_no_noise = acceleration_next_no_noise(a_IDM_next, a_decision_next, params)

    % Get the number of vehicles
    num_vehicles = params.num_vehicles;

    % Check if input vectors have the same length
    if length(a_IDM_next) ~= num_vehicles || length(a_decision_next) ~= num_vehicles
        error('Input vectors a_IDM_next and a_decision_next must have length equal to num_vehicles');
    end

    % Initialize output vector
    a_next_no_noise = zeros(1, num_vehicles);

    for i = 1:num_vehicles
    a_next_no_noise(i) = min(a_IDM_next(i), a_decision_next(i)); 
    end
end
%% acceleration_next

function a_next = acceleration_next(a_IDM_next, a_decision_next, v, params)

    n_a = params.n_a;
    num_vehicles = params.num_vehicles;
    b = params.b;
    a_max = params.a_max;
    dt = params.dt;

    % Input validation
    if length(a_IDM_next) ~= num_vehicles || length(a_decision_next) ~= num_vehicles || length(v) ~= num_vehicles 
        error('Input vectors must have the same length as num_vehicles');
    end

    % Initialize output vector
    a_next = zeros(1, num_vehicles);

    for i = 1:num_vehicles
        left_bound = -n_a * v(i) / dt;
        right_bound = n_a;
        range_width = right_bound - left_bound;
        noise = (rand * range_width) + left_bound; % [-n_a * v(i) / dt, n_a]
        % Calculate next acceleration
        a_next(i) = min(a_IDM_next(i), a_decision_next(i)) + noise;

        % Ensure acceleration is within bounds
        a_next(i) = max(min(a_next(i), a_max), -b);
    end
end
%% acceleration_probability

function p_a = acceleration_probability(a_next, a_IDM_next, a_decision_next, v, params)

    n_a = params.n_a;
    num_vehicles = params.num_vehicles;
    dt = params.dt;

    % Input validation
    if length(a_next) ~= num_vehicles || length(a_IDM_next) ~= num_vehicles ||length(a_decision_next) ~= num_vehicles || length(v) ~= num_vehicles 
        error('Input vectors must have the same length as num_vehicles');
    end

    % Initialize output vector
    p_a = zeros(1, num_vehicles);

    for i = 1:num_vehicles
        a_det = min(a_IDM_next(i), a_decision_next(i));
        left_bound = -n_a * v(i) / dt;
        right_bound = n_a;
        range_width = right_bound - left_bound;
        if a_next(i) >= (a_det + left_bound) && a_next(i) <= (a_det + right_bound)
            p_a(i) = 1 / range_width;
        else
            p_a(i) = 0;
        end
    end
end
%% decision_probability

function p_d = decision_probability(D, D_next, S_next, d_next, d_b_next, d_a_next, params)
    % Inputs:
    %   D: Current decision
    %   D_next: Next decision
    %   S_next: Next signal state
    %   d_next: Next distance to the intersection
    %   d_b_next: Next downstream boundary
    %   d_a_next: Next upstream boundary
    %   params: Struct containing all necessary parameters
    
    % Initialize probability vector
    num_vehicles = params.num_vehicles;
    p_stop = params.p_stop;
    D_stop = params.D_stop;  % Assuming 1 represents D_stop
    D_go = params.D_go;  % Assuming 2 represents D_go
    D_undecided = params.D_undecided; % Assuming 2 represents D_undecided
    d_stop_line = params.d_stop_line;
    D_h = params.D_h;
    p_d = zeros(1, num_vehicles);

    for i = 1:num_vehicles
        % Case 1: Next decision is the same as current decision
        if D_next(i) == D(i)
            p_d(i) = 1;
            continue;
        end

        % Case 2: Vehicle is in the decision zone
        if D_h < d_next(i) && d_next(i) <= d_stop_line
            switch S_next
                case "red"
                    p_d(i) = (D_next(i) == D_stop);
                case "green"
                    p_d(i) = (D_next(i) == D_go);
                case "yellow"
                    if d_next(i) <= d_b_next(i)
                        p_d(i) = (D_next(i) == D_stop);
                    elseif d_a_next(i) <= d_next(i)
                        p_d(i) = (D_next(i) == D_go);
                    else
                        if D(i) == D_undecided
                            p_d(i) = (D_next(i) == D_stop) * p_stop + ...
                               (D_next(i) == D_go) * (1 - p_stop);
                        else
                            p_d(i) = (D_next(i) == D(i));
                        end
                    end
            end
        % Case 3: Vehicle is past the stop line
        elseif d_next(i) > d_stop_line
            p_d(i) = (D_next(i) == D_go);
        % Case 4: Vehicle is before the decision horizon
        elseif d_next(i) <= D_h
            p_d(i) = (D_next(i) == D_undecided);
        end
    end

    % Check for invalid probabilities
    if any(p_d < 0 | p_d > 1)
        error('Invalid probability calculated: %s', mat2str(p_d));
    end
end
%% state_transition_probability

function p_joint = state_transition_probability(p_a, p_d, params)
    % Inputs:
    % p_a: Probability of acceleration (vector)
    % p_d: Probability of decision (vector)

    num_vehicles = params.num_vehicles;
    p_joint = zeros(1, num_vehicles);

    % Input validation
    if length(p_a) ~= num_vehicles || length(p_d) ~= num_vehicles
        error('Input vectors must have the same length as num_vehicles');
    end

    for i = 1:num_vehicles 
    % Calculate joint probability
    p_joint(i) = p_a(i) .* p_d(i);
    end

    % Check for invalid probabilities
    if any(p_joint < 0 | p_joint > 1)
        error('Invalid joint probability calculated: %s', mat2str(p_joint));
    end
end