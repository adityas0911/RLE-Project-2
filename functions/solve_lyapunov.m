function F_lyapunov = solve_lyapunov(A, B1, B2, Q, R1, R2, num_iterations)
    % Initial guess for the Lyapunov equation:
    P = Q; % Initial state cost matrix.
    F_lyapunov = zeros(size(B1, 2), size(A, 1)); % Initialize F_lyapunov.

    % Array to store feedback gains for plotting
    F_lyapunov_vals = zeros(num_iterations, length(F_lyapunov));

    for i = 1:num_iterations
        % Solve the Lyapunov equation iteratively
        P = A' * P * A - (A' * P * B1) * inv(R1 + B1' * P * B1) * (B1' * P * A) + Q; % Lyapunov iteration
        F_lyapunov = inv(R1 + B1' * P * B1) * (B1' * P * A); % Update the Lyapunov feedback gain
        
        % Store the feedback gain for plotting
        F_lyapunov_vals(i, :) = F_lyapunov;

        % Print current value of the feedback gain at each iteration:
        fprintf('Iteration %d: Lyapunov Feedback Gain:\n', i);
        disp(F_lyapunov);
    end

    % Plot the convergence of the Lyapunov feedback gain
    figure;
    plot(1:num_iterations, F_lyapunov_vals(:, 1), 'r', 'LineWidth', 2);
    hold on;
    plot(1:num_iterations, F_lyapunov_vals(:, 2), 'b', 'LineWidth', 2);
    title('Convergence of Lyapunov Feedback Gain');
    legend('F_lyapunov - Component 1', 'F_lyapunov - Component 2');
    xlabel('Iterations');
    ylabel('Feedback Gain');
    grid on;

    % Final Lyapunov feedback gain
    fprintf('Final Lyapunov Feedback Gain:\n');
    disp(F_lyapunov);
end
