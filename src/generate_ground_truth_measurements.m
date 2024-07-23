function [c_tilde, o_tilde, v_avg_tilde, d_tilde] = generate_ground_truth_measurements(params, initial_states)
    % Initialize measurement functions
    M = measurement_functions();

    % Generate ground truth states using simulate1
    [all_states, ~] = simulate1(initial_states, params);

    % Extract necessary parameters
    num_iterations = params.num_iterations;
    num_vehicles = params.num_vehicles;

    % Initialize arrays for measurements
    c = zeros(1, num_iterations);
    c_tilde = zeros(1, num_iterations);
    o = zeros(1, num_iterations);
    o_tilde = zeros(1, num_iterations);
    v_avg = zeros(1, num_iterations);
    v_avg_tilde = zeros(1, num_iterations);
    d_tilde = zeros(num_vehicles, num_iterations);

    % Extract positions and velocities from all_states
    positions = zeros(num_iterations, num_vehicles);
    velocities = zeros(num_iterations, num_vehicles);
    for t = 1:num_iterations
        for v = 1:num_vehicles
            positions(t, v) = all_states{t}(v).d;
            velocities(t, v) = all_states{t}(v).v;
        end
    end

    for t = 1:num_iterations
        % Count vehicles passing the loop
        c(t) = M.count_loop(params, positions(t, :), velocities(t, :));
        % Measure the count with added noise
        c_tilde(t) = M.measure_c(c(t));

        % Presence loop
        o(t) = M.presence_loop(params, positions(t, :), velocities(t, :));
        o_tilde(t) = M.measure_o(o(t), params);

        % Speed loop
        v_avg(t) = M.speed_loop(params, positions(t, :), velocities(t, :));
        v_avg_tilde(t) = M.measure_v_avg(v_avg(t));

        % GPS
        for i = 1:num_vehicles
            d_tilde(i, t) = M.measure_d(positions(t, i), params);
        end
    end
end