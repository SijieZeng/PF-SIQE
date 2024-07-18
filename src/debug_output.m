function debug_output(params, d, v, a, D, c, c_tilde, o, o_tilde, v_avg, v_avg_tilde, d_tilde, t)
    num_vehicles = params.num_vehicles;
    
    if t == 0
        fprintf('Initial state:\n');
    else
        fprintf('Time step %d:\n', t);
        fprintf('c = %.2f, c_tilde = %.2f, o = %d, o_tilde = %d\n,  v_avg = %d, v_avg_tilde = %d\n', c, c_tilde, o, o_tilde, v_avg, v_avg_tilde);
    end
    
    for i = 1:num_vehicles
        fprintf('Vehicle %d: d = %.2f, v = %.2f, a = %.2f, D = %.2f', ...
                i, d(i), v(i), a(i), D(i));
        % Only print d_tilde if it's available (i.e., not in the initial state)
        if t > 0
            fprintf('Vehicle %d: d_tilde = %.2f\n', i, d_tilde(i));
        end
    end
    fprintf('\n');
end