function Measurement = measurement()
    Measurement.calculate_previous_position = @calculate_previous_position;
    Measurement.vehicle_passed_loop = @vehicle_passed_loop;
    Measurement.count_loop = @count_loop;
    Measurement.measure_c  = @measure_c ;
    Measurement.count_loop_probability = @count_loop_probability;
    Measurement.presence_loop = @presence_loop;
    Measurement.measure_o  = @measure_o ;
    Measurement.presence_loop_probability = @presence_loop_probability;
    Measurement.speed_loop = @speed_loop;
    Measurement.measure_v_avg = @measure_v_avg;
    Measurement.speed_loop_probability = @speed_loop_probability;
    Measurement.measure_d = @measure_d;
    Measurement.GPS_probability = @GPS_probability;
    Measurement.measurement_probability = @measurement_probability;
end
%% calculate_previous_position
function d_previous = calculate_previous_position(d, v, params, loop_type)
    switch loop_type
        case 'loop1'
            dt = params.dt_loop1;
        case 'loop2'
            dt = params.dt_loop2;
        case 'loop3'
            dt = params.dt_loop3;
        otherwise
            error('Invalid loop type specified');
    end
    d_previous = d - v * dt;
end
%% vehicle_passed_loop

function [passed, d_previous] = vehicle_passed_loop(d, v, params, loop_type, loop_position)
assert(isnumeric(d) && isnumeric(v) && isstruct(params) && ischar(loop_type) && isnumeric(loop_position), ...
        'Invalid input types');
    % Calculate previous position using the updated function
    d_previous = calculate_previous_position(d, v, params, loop_type);
    
    % Check if the vehicle passed the loop
    passed = (d >= loop_position) && (d_previous <= loop_position);
    % Debug output
    fprintf('vehicle_passed_loop: d=%.2f, v=%.2f, previous_position=%.2f, loop_position=%.2f, passed=%d\n', ...
            d, v, d_previous, loop_position, passed);
end
%% count_loop

function c = count_loop(params, d, v)

    % Get the time step
    dt = params.dt_loop1;
    d_loop1 = params.d_loop1;
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;

    % Initialize an array to store counts at each time step
    c = zeros(1, num_iterations);
    d_previous = zeros(1, num_vehicles);

    % Loop through each time step
    for t = 1:num_iterations
        % Check each vehicle
        for i = 1:num_vehicles
            % Calculate position at current and next time step
            d_previous(i) = d(i) - v(i) * dt;
            
            % Check if vehicle passed the loop during this time step
            if (d_previous(i) < d_loop1) && (d(i) >= d_loop1)
                c(t) = c(t) + 1;
            end
        end
    end
end
%% measure_c

function c_tilde = measure_c(c, params)
    % Measure the vehicle count with added noise
    % c : actual vehicle count
    % c _tilde: measured vehicle count with noise

    num_iterations = params.num_iterations;

    c_tilde = zeros(1, num_iterations);

    for t = 1:num_iterations
        % Calculate the standard deviation of the noise
        sigma = 10.2 * sqrt(c(t)  / 1000);
    
        % Generate noise from a normal distribution
        n_loop1 = normrnd(0, sigma);
    
        % Add noise to the actual count
        c_tilde(t) = c(t)  + n_loop1;
    
        % Ensure the measured count is non-negative
        c_tilde(t) = max(0, round(c_tilde(t)));
    end
end
%% count_loop_probability

function p_c = count_loop_probability(c_tilde, c, params)
    % Calculate the probability density for the measured vehicle count
    % c _tilde: measured vehicle count
    % c : actual vehicle count
    % p: probability density

    num_iterations = params.num_iterations;
    p_c = zeros(1, num_iterations);

    for t = 1:num_iterations
        % Calculate the standard deviation
        sigma = 10.2 * sqrt(c(t) / 1000);
    
        % Calculate the probability density using the normal distribution formula
        p_c(t) = (1 / (sqrt(2 * pi) * sigma)) * exp(-(c_tilde(t) - c(t) )^2 / (2 * sigma^2));
    end
end
%% presence_loop

function o = presence_loop(params, d, v)
    % Detect vehicle presence at loop 2
    o = 0;
    for i = 1:params.num_vehicles
        [passed, ~] = vehicle_passed_loop(d(i), v(i), params, 'loop2', params.d_loop2);
        if passed
            o = 1;
            break;  % Exit the loop as presence is detected
        end
    end
end
%% measure_o

function o_tilde = measure_o(o, params)
    % Simulate the measurement of vehicle presence with potential errors
    % o : actual vehicle presence (0 or 1)
    % accuracy: probability of correct measurement (between 0 and 1)
    % o _tilde: measured vehicle presence (0 or 1)

    if ~isfield(params, 'accuracy_loop2')
        error('params.accuracy_loop2 is not defined');
    end
    accuracy = params.accuracy_loop2;
    if accuracy < 0 || accuracy > 1
        error('Accuracy must be between 0 and 1');
    end

    % Generate a random value from uniform distribution [0,1]
    r = rand();

    % Determine the measurement based on accuracy
    if r < accuracy
        % Measurement is correct
        o_tilde = o ;
    else
        % Measurement is incorrect
        o_tilde = 1 - o ;  % Flip the bit
    end
end
%% presence_loop_probability

function p_o = presence_loop_probability(o_tilde, o, params)
    % Calculate the probability for the measured vehicle presence
    % o _tilde: measured vehicle presence (0 or 1)
    % o : actual vehicle presence (0 or 1)
    % accuracy: probability of correct measurement (p in the equation)
    % p: probability of the measurement given the actual state

    accuracy = params.accuracy_loop2;

    % Check if the measurement matches the actual state
    if o_tilde == o 
        p_o = accuracy;
    else
        p_o = 1 - accuracy;
    end
end
%% speed_loop

function v_avg = speed_loop(params, d, v)
    % Calculate average speed of vehicles passing through loop 3
    
    v_sum = 0;
    n = 0;
    
    for i = 1:params.num_vehicles
        [passed, ~] = vehicle_passed_loop(d(i), v(i), params, 'loop3', params.d_loop3);
        if passed
            v_sum = v_sum + v(i);
            n = n + 1;
        end
    end
    
    if n > 0
        v_avg = v_sum / n;
    else
        v_avg = 0;  % No vehicles passed the loop
    end
end
%% measure_v_avg

function v_avg_tilde = measure_v_avg(v_avg)
     % Measure the vehicle speed with added noise
    %
    % Inputs:
    %   v_avg: Actual average vehicle speed (m/s)
    %
    % Outputs:
    %   v_avg_tilde: Measured average vehicle speed with noise (m/s)
    %
    % Note: This function simulates measurement noise based on the speed range
    

    % Determine which noise model to use (95% or 5% chance)
    if rand() <= 0.95
        % 95% of the cases
        if v_avg <= 5.6
            noise = unifrnd(-0.1 * v_avg, 0.1 * v_avg);
        elseif v_avg <= 16.7
            noise = unifrnd(-0.03 * v_avg, 0.03 * v_avg);
        elseif v_avg <= 50
            noise = unifrnd(-0.05 * v_avg, 0.05 * v_avg);
        elseif v_avg < 69.4
            noise = unifrnd(-0.1 * v_avg, 0.1 * v_avg);
        else
            warning('Speed exceeds maximum range. Clamping to 69.4 m/s.');
            v_avg = 69.4;
        end
    else
        % 5% of the cases
        if v_avg <= 5.6
            noise = unifrnd(-0.2 * v_avg, 0.2 * v_avg);
        elseif v_avg <= 16.7
            noise = unifrnd(-0.06 * v_avg, 0.06 * v_avg);
        elseif v_avg <= 50
            noise = unifrnd(-0.1 * v_avg, 0.1 * v_avg);
        elseif v_avg < 69.4
            noise = unifrnd(-0.2 * v_avg, 0.2 * v_avg);
        else
            warning('Speed exceeds maximum range. Clamping to 69.4 m/s.');
            v_avg = 69.4;
        end
    end

    % Add noise to the actual speed
    v_avg_tilde = v_avg + noise;
end
%% speed_loop_probability

function p_s = speed_loop_probability(v_avg_tilde, v_avg)
    % Calculate the probability density for the measured vehicle speed
    % v_avg_tilde: measured vehicle speed in m/s
    % v_avg: actual vehicle speed in m/s
    % p: probability density

    if v_avg <= 5.6
        if 0.9 * v_avg <= v_avg_tilde && v_avg_tilde <= 1.1 * v_avg
            p_s = 0.95 / (0.2 * v_avg);
        elseif (0.8 * v_avg <= v_avg_tilde && v_avg_tilde < 0.9 * v_avg) || (1.1 * v_avg < v_avg_tilde && v_avg_tilde <= 1.2 * v_avg)
            p_s = 0.05 / (0.4 * v_avg);
        else
            p_s = 0;
        end
    elseif v_avg <= 16.7
        if 0.97 * v_avg <= v_avg_tilde && v_avg_tilde <= 1.03 * v_avg
            p_s = 0.95 / (0.06 * v_avg);
        elseif (0.94 * v_avg <= v_avg_tilde && v_avg_tilde < 0.97 * v_avg) || (1.03 * v_avg < v_avg_tilde && v_avg_tilde <= 1.06 * v_avg)
            p_s = 0.05 / (0.12 * v_avg);
        else
            p_s = 0;
        end
    elseif v_avg <= 50
        if 0.95 * v_avg <= v_avg_tilde && v_avg_tilde <= 1.05 * v_avg
            p_s = 0.95 / (0.1 * v_avg);
        elseif (0.9 * v_avg <= v_avg_tilde && v_avg_tilde < 0.95 * v_avg) || (1.05 * v_avg < v_avg_tilde && v_avg_tilde <= 1.1 * v_avg)
            p_s = 0.05 / (0.2 * v_avg);
        else
            p_s = 0;
        end
    elseif v_avg < 69.4
        if 0.9 * v_avg <= v_avg_tilde && v_avg_tilde <= 1.1 * v_avg
            p_s = 0.95 / (0.2 * v_avg);
        elseif (0.8 * v_avg <= v_avg_tilde && v_avg_tilde < 0.9 * v_avg) || (1.1 * v_avg < v_avg_tilde && v_avg_tilde <= 1.2 * v_avg)
            p_s = 0.05 / (0.4 * v_avg);
        else
            p_s = 0;
        end
    else
        error('Speed out of range');
    end
end
%% 

function d_tilde = measure_d(d, params)
    % Measure the vehicle position with GPS noise
    % d: actual vehicle position, from function next_state = nextState(state, params)
    % params: structure containing necessary parameters (including sigma_GPS)
    % d_tilde: measured vehicle position

    % Extract GPS measurement standard deviation
    sigma_GPS = params.sigma_GPS;

    % Generate noise from a normal distribution
    n_GPS = normrnd(0, sigma_GPS);

    % Add noise to the actual position
    d_tilde = d + n_GPS;
end
%% 

function p_G = GPS_probability(d_tilde, d, params)
    % Calculate the probability density for the measured vehicle position
    % d_tilde: measured vehicle position
    % d: actual vehicle position
    % params: structure containing necessary parameters (including sigma_GPS)
    % p: probability density

    % Extract GPS measurement standard deviation
    sigma_GPS = params.sigma_GPS;

    % Calculate the probability density using the normal distribution formula
    p_G = (1 / (sqrt(2 * pi) * sigma_GPS)) * exp(-(d_tilde - d)^2 / (2 * sigma_GPS^2));
end
%% 

function p = measurement_probability(c_tilde, c, o_tilde, o, v_avg_tilde, v_avg, d_tilde, d, params)
    % Calculate the joint probability of loop measurements given the state
    % params: structure containing necessary parameters
    % p: joint probability of the measurements
    assert(isnumeric(c_tilde) && isnumeric(c) && isnumeric(o_tilde) && isnumeric(o) && ...
           isnumeric(v_avg_tilde) && isnumeric(v_avg) && isnumeric(d_tilde) && isnumeric(d) && ...
           isstruct(params), 'Invalid input types');

    % Calculate individual probabilities
    p_loop1 = count_loop_probability(c_tilde, c);
    p_loop2 = presence_loop_probability(o_tilde, o, params);
    p_loop3 = speed_loop_probability(v_avg_tilde, v_avg);
    p_GPS = GPS_probability(d_tilde, d, params);

    % Calculate joint probability
    p = p_loop1 * p_loop2 * p_loop3 * p_GPS;
end