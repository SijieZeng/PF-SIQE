function M = measurement()
    M.calculate_previous_position = @calculate_previous_position;
    M.count_loop = @count_loop;
    M.measure_c  = @measure_c ;
    M.count_loop_probability_density = @count_loop_probability_density;
    M.presence_loop = @presence_loop;
    M.measure_o  = @measure_o ;
    M.presence_loop_probability = @presence_loop_probability;
    M.speed_loop = @speed_loop;
    M.measure_v_avg = @measure_v_avg;
    M.speed_loop_probability = @speed_loop_probability;
    M.measure_d = @measure_d;
    M.GPS_probability = @GPS_probability;
    M.measurement_probability = @measurement_probability;
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
%% count_loop

function count = count_loop(params, d, v)

    % Get the time step
    dt = params.dt_loop1;
    d_loop1 = params.d_loop1;
    num_vehicles = params.num_vehicles;

    % Initialize an array to store counts at each time step
    count = 0;
    d_previous = zeros(1, num_vehicles);

    % Loop through  each vehicle
    for i = 1:num_vehicles
        % Calculate position at current and next time step
        d_previous(i) = d(i) - v(i) * dt;
            
        % Check if vehicle passed the loop during this time step
        if (d_previous(i) < d_loop1) && (d(i) >= d_loop1)
            count = count + 1;
        end     
    end
end
%% measure_c

function c_tilde = measure_c(c)
    % Measure the vehicle count with added noise
    % c : actual vehicle count
    % c _tilde: measured vehicle count with noise

      % Calculate the standard deviation of the noise
        sigma = 10.2 * sqrt(c / 1000);
    
        % Generate noise from a normal distribution
        n_loop1 = normrnd(0, sigma);
    
        % Add noise to the actual count
        c_tilde=  c + n_loop1;
    
        % Ensure the measured count is non-negative
        c_tilde= max(0, c_tilde);
    end

%% count_loop_probability

function p_c = count_loop_probability_density(c_tilde, c)
    % Calculate the probability density for the measured vehicle count
    % c _tilde: measured vehicle count
    % c : actual vehicle count
    % p: probability density

    c = max(c, 0);
    c_tilde = max(c_tilde, 0);

    if c == 0
        % 当实际计数为 0 时，使用一个很小的常数值作为 sigma
        sigma = 0.5;  % 这个值可以根据实际情况调整
    else
         sigma = max(10.2 * sqrt(c / 1000), 0.5); % 确保 sigma 不会太小
    end
     % 使用正态分布公式计算概率密度
    exponent = -(c_tilde - c)^2 / (2 * sigma^2);
    % Calculate the probability density using the normal distribution formula
    p_c = (1 / (sqrt(2 * pi) * sigma)) * exp(exponent);
 end

%% presence_loop

function o = presence_loop(params, d, v)

    dt = params.dt_loop2;
    d_loop2 = params.d_loop2;
    num_vehicles = params.num_vehicles;
    % Detect vehicle presence at loop 2
    o = 0;
    d_previous = zeros(1, num_vehicles);

    for i = 1:params.num_vehicles
        % Calculate position at current and next time step
        d_previous(i) = d(i) - v(i) * dt;
            
        % Check if vehicle passed the loop during this time step
        if (d_previous(i) < d_loop2) && (d(i) >= d_loop2)
            o = 1;
            break;
        end
    end
end
%% measure_o

function o_tilde = measure_o(o, params)
    % Simulate the measurement of vehicle presence with potential errors
    % o : actual vehicle presence (0 or 1)
    % accuracy: probability of correct measurement (between 0 and 1)
    % o _tilde: measured vehicle presence (0 or 1)

    accuracy = params.accuracy_loop2;

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
    dt = params.dt_loop3;
    d_loop3 = params.d_loop3;
    num_vehicles = params.num_vehicles;
    v_sum = 0;
    n = 0;
    d_previous = zeros(1, num_vehicles);
    
    for i = 1:num_vehicles
        % Calculate position at current and next time step
        d_previous(i) = d(i) - v(i) * dt;
            
        % Check if vehicle passed the loop during this time step
        if (d_previous(i) < d_loop3) && (d(i) >= d_loop3)
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
%% measure_d

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
%% GPS_probability

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
%% measurement_probability

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