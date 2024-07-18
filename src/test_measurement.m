% Initialize the necessary classes
ST = state_transition_functions();
M = measurement_functions();

% Set up parameters
params = struct();
params.dt = 1;  % Time step for nextState
params.dt_loop1 = 1;  % Time step for count_loop (should match dt)
params.dt_loop2 = 1; % Time step for presence_loop
params.dt_loop3 = 1; 
params.d_loop1 = 200;  % Loop position
params.d_loop2 = 250; 
params.d_loop3 = 220;% Loop 2 position
params.num_vehicles = 3;
params.num_iterations = 5;  % Increase iterations to see vehicles pass the loop
params.accuracy_loop2 = 0.95; % 95% accuracy for presence detection
params.sigma_GPS = 5; % GPS测量的标准差，单位可能是米

% Initial states for vehicles
% Format: [d, v, a, D, lane]
initial_states = [
    215, 10, 1, 0 ;  % This vehicle will pass the loop
    180, 5, 0, 0;   % This vehicle will not pass the loop in 5 iterations
    150, 5, 0, 0    % This vehicle will pass the loop
];
% Extract initial values for d, v, a, D, and lane
d = initial_states(:, 1)';
v = initial_states(:, 2)';
a = initial_states(:, 3)';
D = initial_states(:, 4)';


debug_output(params, d, v, a, D, 0, 0, 0, 0, 0, 0, 0, 0);

% Run simulation and count vehicles
c = zeros(1, params.num_iterations);
c_tilde = zeros(1, params.num_iterations);
o = zeros(1, params.num_iterations);
o_tilde = zeros(1, params.num_iterations);
v_avg = zeros(1, params.num_iterations);
v_avg_tilde = zeros(1, params.num_iterations);
d_tilde = zeros(params.num_vehicles, params.num_iterations);
current_states = initial_states;

for t = 1:params.num_iterations
    % Get next states
    next_states = ST.nextState(current_states, params);

    % Extract d, v, a, D, and lane for debug_output
    d = next_states(:, 1)'; % Transpose to row vector
    v = next_states(:, 2)'; % Transpose to row vector
    a = next_states(:, 3)';
    D = next_states(:, 4)';
    
    % Count vehicles passing the loop
    c(t) = M.count_loop(params, d, v);
   
    % Measure the count with added noise
    c_tilde(t) = M.measure_c(c(t));

    % Presence loop
    o(t) = M.presence_loop(params, d, v);
    o_tilde(t) = M.measure_o(o(t), params);

    % speed loop
    v_avg(t) = M.speed_loop(params, d, v);
    v_avg_tilde(t) = M.measure_v_avg(v_avg(t));

    % GPS
    for i = 1:params.num_vehicles
        d_tilde(i, t) = M.measure_d(d(i), params);
    end

    % Output debug information
    debug_output(params, d, v, a, D, c(t), c_tilde(t), o(t), o_tilde(t), v_avg(t), v_avg_tilde(t),d_tilde(:, t), t);
    
    % Update current states for next iteration
    current_states = next_states;

    pdf = M.count_loop_probability_density(c_tilde(t), c(t));
    p_o = M.presence_loop_probability(o_tilde(t), o(t), params);
    p_s = M.speed_loop_probability(v_avg_tilde(t), v_avg(t));
    p_G = M.GPS_probability(d_tilde(i, t), d(i), params);
    %fprintf('Time step %d:\n', t);
    fprintf('  Count loop - Probability density: %.4f\n', pdf);
    fprintf('  Presence loop - Probability: %.4f\n', p_o);
    fprintf('  Speed loop - Probability: %.4f\n', p_s);
    fprintf('  GPS - Probability: %.4f\n', p_G);
end



    