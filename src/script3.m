% Initialize the necessary classes
ST = state_transition();
M = measurement();

% Set up parameters
params = struct();
params.dt = 1;  % Time step for nextState
params.dt_loop1 = 1;  % Time step for count_loop (should match dt)
params.d_loop1 = 200;  % Loop position
params.num_vehicles = 3;
params.num_iterations = 5;  % Increase iterations to see vehicles pass the loop

% Initial states for vehicles
% Format: [d, v, a, D, lane]
initial_states = [
    190, 10, 1, 0, 1;  % This vehicle will pass the loop
    180, 5, 0, 0, 1;   % This vehicle will not pass the loop in 5 iterations
    150, 5, 0, 0, 1    % This vehicle will pass the loop
];

% Run simulation and count vehicles
counts = zeros(1, params.num_iterations);
measured_counts = zeros(1, params.num_iterations);
current_states = initial_states;

for t = 1:params.num_iterations
    % Get next states
    next_states = ST.nextState(current_states, params);
    
    % Extract d and v for count_loop
    d = next_states(:, 1)';  % Transpose to row vector
    v = next_states(:, 2)';  % Transpose to row vector
    
    % Count vehicles passing the loop
    c = M.count_loop(params, d, v);
    counts(t) = c(1);  % We only need the first (and only) element of c

    % Measure the count with added noise
    c_tilde = M.measure_c(counts(t), params);
    measured_counts(t) = c_tilde(1); % We only need the first element
    
    % Update current states for next iteration
    current_states = next_states;
    
    % Print current positions and speeds
    fprintf('Time step %d:\n', t);
    for i = 1:params.num_vehicles
        fprintf('Vehicle %d - Position: %.2f, Speed: %.2f\n', i, d(i), v(i));
    end
    fprintf('Vehicles passing loop: %d\n\n', counts(t));
    fprintf('Measured vehicles passing loop: %d\n\n', measured_counts(t));
end


% Print final results
fprintf('Total actual vehicles detected: %d\n', sum(counts));
fprintf('Total measured vehicles detected: %d\n', sum(measured_counts));

% Calculate and print the probability for each measurement
for t = 1:params.num_iterations
    p = M.count_loop_probability(measured_counts(t), counts(t));
    fprintf('Time step %d - Probability of measurement: %.4f\n', t, p);
end