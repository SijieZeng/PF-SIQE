function StateTransition = state_transition()
    StateTransition.nextState = @nextState;
    StateTransition.calculate_vehicle_relations = @calculate_vehicle_relations;
    StateTransition.intelligent_driver_model = @intelligent_driver_model;
    StateTransition.generate_traffic_signal_states = @generate_traffic_signal_states;
    StateTransition.update_elapsed_time = @update_elapsed_time;
    StateTransition.calculate_T_elapsed_next = @calculate_T_elapsed_next;
    StateTransition.decision_making = @decision_making;
    StateTransition.traffic_light_decision_model = @traffic_light_decision_model;
    StateTransition.acceleration_next = @acceleration_next;
    StateTransition.acceleration_probability = @acceleration_probability;
    StateTransition.decision_probability = @decision_probability;
    StateTransition.state_transition_probability = @state_transition_probability;
    %StateTransition.groundtruth = @groundtruth;
end
%% nextState

function next_state = nextState(state, params)
    d = state(1); % Longitudinal Position (m)
    v = state(2); % Speed (m/s)
    a = state(3); % Acceleration (m/s^2)
    D = state(4); % Decision of stop or go when facing the traffic light
    lane = state(5); % which Lane

    T = params.dt;

     % Ensure acceleration and time step are valid
    if T <= 0 || isnan(T) || isnan(a) || isinf(T) || isinf(a)
        error('Invalid parameters: T: %f, a: %f', T, a);
    end

    v = max(0, v + a * T);



    d = d + v * T + 0.5 * a * T^2;

    if isnan(d) || isnan(v) || isinf(d) || isinf(v)
        error('Invalid state detected. d: %f, v: %f', d, v);
    end

    next_state = [d, v, a, D, lane];
end
%% calculate_vehicle_relations

function [delta_v, s] = calculate_vehicle_relations(d, v, params)
    if ~isstruct(params)
        error('params must be a struct');
    end
    if ~isfield(params, 'num_vehicles')
        error('params.num_vehicles is not defined');
    end
    num_vehicles = params.num_vehicles;
    s0 = params.s0;  % Minimum gap

    delta_v = zeros(1, num_vehicles);
    s = zeros(1, num_vehicles);
    
    % Add debug output
    fprintf('Debug: d = %s\n', mat2str(d));
    fprintf('Debug: v = %s\n', mat2str(v));
    
    for i = 2:num_vehicles
        delta_v(i) = v(i-1) - v(i);
        s(i) = max(s0, d(i-1) - d(i)); % Ensure non-negative spacing
        
        % Add more debug output
        fprintf('Debug: Vehicle %d, delta_v = %.4f, s = %.4f\n', i, delta_v(i), s(i));
    end
    
    % Lead vehicle
    delta_v(1) = 0;
    s(1) = Inf;
    
    % Check for invalid values, excluding the lead vehicle's s value
    epsilon = 1e-10;  % Small tolerance for floating-point comparisons
    if any(isnan(delta_v)) || any(isnan(s)) || any(isinf(delta_v)) || any(s(2:end) < (s0 - epsilon))
        error('Invalid values detected in vehicle relations calculations.');
    end

    % Add more detailed error reporting
    if any(isnan(delta_v))
        error('NaN values detected in delta_v');
    elseif any(isnan(s))
        error('NaN values detected in s');
    elseif any(isinf(delta_v))
        error('Inf values detected in delta_v');
    elseif any(s(2:end) < (s0 - epsilon))
        error('s values less than s0 detected: %s', mat2str(s(2:end)));
    end
end
%% intelligent_driver_model

function a_IDM_next = intelligent_driver_model(v, delta_v, s, params)
  % Intelligent Driver Model (IDM) function
    % Inputs:
    %   v - current speed of the vehicle
    %   delta_v - speed difference to the leading vehicle
    %   s - gap to the leading vehicle
    %   params - struct containing IDM parameters
    % Outputs:
    %   a_IDM_next - computed acceleration at time step k+1

    v_desired = params.v_desired;
    a_max = params.a_max;
    b = params.b;
    s0 = params.s0;
    T = params.T; % safe time headway
    delta = params.delta;

    % Desired gap
    s_star = s0 + v * T + (v * delta_v) / (2 * sqrt(a_max * b));

    % Acceleration
    a_IDM_next = a_max * (1 - (v / v_desired)^delta - (s_star / s)^2);
    
    % Ensure acceleration is within reasonable bounds
    a_IDM_next = max(min(a_IDM_next, params.a_max), -params.b);
    
    % Check for invalid values
    if isnan(a_IDM_next) || isinf(a_IDM_next)
        error('Invalid acceleration detected. a_IDM_next: %f', a_IDM_next);
    end
end
%% generate_traffic_signal_states

function S = generate_traffic_signal_states(params)
    num_iterations = params.num_iterations;
    red_time = params.red_time;   
    yellow_time = params.yellow_time; 
    green_time = params.green_time; 
    S = strings(num_iterations, 1);
    cycle_length = red_time + yellow_time + green_time;

    for k = 1:num_iterations
        cycle_position = mod(k-1, cycle_length);

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

function T_elapsed = update_elapsed_time(S, params)
    % Update elapsed yellow light time for each vehicle
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;
    T_elapsed = zeros(num_iterations, num_vehicles); % Initialize elapsed time matrix
    yellow_time = params.yellow_time; % Yellow light duration
    dt = params.dt; % Time step
    

    for k = 1:num_iterations
        if k > 1
            if S(k) == "yellow"
                if S(k-1) ~= "yellow"
                    % Start of yellow light
                    T_elapsed(k, :) = yellow_time;
                else
                    % During yellow light
                    T_elapsed(k, :) = max(T_elapsed(k-1, :) - dt, 0);
                end
            elseif S(k) == "red" && S(k-1) == "yellow"
                % End of yellow light
                T_elapsed(k, :) = 0;
            else
                % Green light or continued red light
                T_elapsed(k, :) = 0;
            end
        end
    end
end
%% calculate_T_elapsed_next

function T_elapsed_next = calculate_T_elapsed_next(T_elapsed, S, S_next, params)
    yellow_time = params.yellow_time; % Yellow light duration
    dt = params.dt; % Time step
    num_vehicles = params.num_vehicles; % Number of vehicles
    T_elapsed_next = zeros(1, num_vehicles); % Initialize T_elapsed_next as an array

    for i = 1:num_vehicles
        if S == "yellow" && S_next == "yellow"
            % During yellow light
            T_elapsed_next(i) = max(T_elapsed(i) - dt, 0);
        elseif S ~= "yellow" && S_next == "yellow"
            % Start of yellow light
            T_elapsed_next(i) = yellow_time;
        else
            % End of yellow light or not in yellow light
            T_elapsed_next(i) = 0;
        end
    end
end
%% decision_making

function D_next = decision_making(d_k, d_next, v_k, v_next, S_k, S_next, T_elapsed, T_elapsed_next, D_k, params)
% Decision-making function based on current state and signal state
    % Inputs:
    %   d_k - position at time k
    %   d_next - position at time k+1
    %   v_k - speed at time k
    %   v_next - speed at time k+1
    %   S_k - signal state at time k
    %   S_next - signal state at time k+1
    %   T_elapsed - elapsed yellow time at time k
    %   T_elapsed_next - elapsed yellow time at time k+1
    %   D_k - decision at time k
    %   num_vehicles - number of vehicles
    %   params - struct containing all parameters
    % Output:
    %   D_k1 - decision at time k+1

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
        if d_0 <= d_next(i) && d_next(i) <= D_h
            D_next(i) = params.D_undecided; % D_undecided
        elseif D_h <= d_next(i) && d_next(i) <= d_stop_line
            if S_next == "red"
                D_next(i) = params.D_stop; % D_stop
            elseif S_next == "green"
                D_next(i) = params.D_go; % D_go
            elseif S_next == "yellow"
                d_a_next = T_reaction * v_next(i) + (v_next(i)^2) / (2 * b);
                % Debugging information
                fprintf('Debug: T_elapsed_next = %s\n', mat2str(T_elapsed_next));
                fprintf('Debug: v_next = %s\n', mat2str(v_next));
                fprintf('Debug: i = %d\n', i);
                d_b_next = (T_yellow - T_elapsed_next(i)) * v_next(i);

                if d_next(i) <= d_b_next
                    D_next(i) = params.D_stop; % D_stop
                elseif d_a_next <= d_next(i) 
                    D_next(i) = params.D_go; % D_go
                else % d_b_next < d_next(i) && d_next(i) < d_a_next
                    if D_k(i) == params.D_undecided % D_undecided
                        r = rand();
                        if r < p
                            D_next(i) = params.D_stop; % D_stop
                        else
                            D_next(i) = params.D_go; % D_go
                        end
                    else
                        D_next(i) = D_k(i);
                    end
                end
            end
        else
            D_next(i) = params.D_go;
        end

        % Maintain previous decision if already made during yellow light
        if S_k == "yellow" && S_next == "yellow" && D_k(i) ~= params.D_undecided
            d_b_k = (T_yellow - T_elapsed(i)) * v_k(i);
            d_a_k = T_reaction * v_k(i) + (v_k(i)^2) / (2 * b);
            d_b_next = (T_yellow - T_elapsed_next(i)) * v_next(i);
            d_a_next = T_reaction * v_next(i) + (v_next(i)^2) / (2 * b);

            if d_b_k < d_k(i) && d_k(i) < d_a_k && d_b_next < d_next(i) && d_next(i) < d_a_next
                D_next(i) = D_k(i);
            end
        end
    end
end
%% traffic_light_decision_model

function a_decision_next = traffic_light_decision_model(D_k, v_k, d_k, params)
    % Input:
    %   D_k: Current decision (D_stop, D_go, or D_undecided)
    %   v_k: Current velocity
    %   d_k: Current distance to the intersection
    %   params: Struct containing all necessary parameters
    
    % Extract parameters
    T_discrete = params.T_discrete;  % Discretization time interval (1 s)
    a_prime = params.a;  % Comfortable acceleration (1.0 m/s^2)
    v_desired = params.v_desired;  % Desired speed
    
    % Calculate C_s,k
    C_s_k = max(1, 2 * d_k / (v_k * T_discrete));
    
    % Determine a_decision_next based on the current decision and conditions
    if D_k == params.D_stop
        a_decision_next = -v_k / (C_s_k * T_discrete);
    elseif D_k == params.D_go && v_k > v_desired
        a_decision_next = 0;
    else
        a_decision_next = a_prime * (1 - (v_k / v_desired)^2);
    end
end
%% acceleration_next

function a_next = acceleration_next(a_IDM_next, a_decision_next, params, v_k)
    n_a = params.n_a;
    T = params.T;
    
    % New noise distribution
    lower_bound = -v_k / T;
    upper_bound = n_a;
    
    noise = (rand * (upper_bound - lower_bound) + lower_bound);
    a_next = min(a_IDM_next, a_decision_next) + noise;
end

function p = acceleration_probability(a_next, params, v_k)
    n_a = params.n_a;
    T = params.T;
    
    % New noise distribution bounds
    lower_bound = -v_k / T;
    upper_bound = n_a;
    
    % Calculate the range of the uniform distribution
    range = upper_bound - lower_bound;
    
    if a_next >= (a_next + lower_bound) && a_next <= (a_next + upper_bound)
        p = 1 / range;
    else
        p = 0;
    end
end
%% decision_probability

function p = decision_probability(D_k, D_next, S_next, d_next, d_b_next, d_a_next, params)
    % Inputs:
    %   D_k: Current decision
    %   D_next: Next decision
    %   S_next: Next signal state
    %   d_next: Next distance to the intersection
    %   d_b_next: Next downstream boundary
    %   d_a_next: Next upstream boundary
    %   params: Struct containing all necessary parameters
    
    % Extract parameters
    p_stop = params.p_stop; % Probability of stopping when undecided
    
    % Initialize probability
    p = 0;
    
    % Case 1: Next decision is the same as current decision
    if D_next == D_k
        p = 1;
    % Case 2: Yellow light, in dilemma zone, currently undecided
    elseif S_next == "yellow" && d_b_next < d_next && d_next < d_a_next && D_k == params.D_undecided
        if D_next == params.D_stop
            p = p_stop;
        elseif D_next == params.D_go
            p = 1 - p_stop;
        end
    end
    
    % Check for invalid probability
    if p < 0 || p > 1
        error('Invalid probability calculated: %f', p);
    end
end
%% state_transition_probability

function p_joint = state_transition_probability(~, a_next, D_k, D_next, S_next, d_next, d_b_next, d_a_next, params)
    % Inputs:
    %   a_k: Current acceleration
    %   a_next: Next acceleration
    %   D_k: Current decision
    %   D_next: Next decision
    %   S_next: Next signal state
    %   d_next: Next distance to the intersection
    %   d_b_next: Next downstream boundary
    %   d_a_next: Next upstream boundary
    %   params: Struct containing all necessary parameters
    
    % Calculate probability of acceleration
    p_acceleration = acceleration_probability(a_next, params);
    
    % Calculate probability of decision
    p_decision = decision_probability(D_k, D_next, S_next, d_next, d_b_next, d_a_next, params);
    
    % Calculate joint probability
    p_joint = p_acceleration * p_decision;
    
    % Check for invalid probability
    if p_joint < 0 || p_joint > 1
        error('Invalid joint probability calculated: %f', p_joint);
    end
end




