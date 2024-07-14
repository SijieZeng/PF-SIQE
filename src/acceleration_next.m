%% acceleration_next

function a_next = acceleration_next(a_IDM_next, a_decision_next, params, v)

    n_a = params.n_a;
    dt = params.dt;
    num_vehicles = params.num_vehicles;
    b = params.b;
    a_max = params.a_max;

    % Input validation
    if length(a_IDM_next) ~= num_vehicles || length(a_decision_next) ~= num_vehicles || length(v) ~= num_vehicles
        error('Input vectors must have the same length as num_vehicles');
    end

    % Initialize output vector
    a_next = zeros(1, num_vehicles);

    for i = 1:num_vehicles
        % New noise distribution
        lower_bound = -v(i) / dt;
        upper_bound = n_a;
    
        noise = (rand * (upper_bound - lower_bound) + lower_bound);
        % Calculate next acceleration
        a_next(i) = min(a_IDM_next(i), a_decision_next(i)) + noise;

        % Ensure acceleration is within bounds
        a_next(i) = max(min(a_next(i), a_max), -b);
    end
end
%% acceleration_probability

function p = acceleration_probability(a_next, params, v)

    n_a = params.n_a;
    dt = params.dt;
    num_vehicles = params.num_vehicles;

    % Input validation
    if length(a_next) ~= num_vehicles || length(v) ~= num_vehicles
        error('Input vectors must have the same length as num_vehicles');
    end

    % Initialize output vector
    p = zeros(1, num_vehicles);

    for i = 1:num_vehicles
        % New noise distribution bounds
        lower_bound = -v(i) / dt;
        upper_bound = n_a;
    
        % Calculate the range of the uniform distribution
        range = upper_bound - lower_bound;

        if range == 0
            p(i) = 0;
        elseif a_next(i) >= (a_next(i) + lower_bound) && a_next(i) <= (a_next(i) + upper_bound)
            p(i) = 1 / range;
        else
            p(i) = 0;
        end
    end
end