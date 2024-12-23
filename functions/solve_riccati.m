function [F1, F2] = solve_riccati(A, B1, B2, Q1, Q2, R1, R2, num_iterations)
    % Initialize the Riccati equations for each drone:
    P1 = Q1; % Initial state cost matrix for Drone 1.
    P2 = Q2; % Initial state cost matrix for Drone 2.

    % Initialize F1 and F2 before using them in the loop
    F1 = zeros(size(B1, 2), size(A, 1)); % Initialize F1 (1x4 vector)
    F2 = zeros(size(B2, 2), size(A, 1)); % Initialize F2 (1x4 vector)

    % Arrays to store feedback gains over iterations
    F1_vals = zeros(num_iterations, length(F1));
    F2_vals = zeros(num_iterations, length(F2));

    for i = 1:num_iterations
        % Solve the Riccati equation for each drone iteratively:
        P1 = A' * P1 * A - (A' * P1 * B1) * inv(R1 + B1' * P1 * B1) * (B1' * P1 * A) + Q1;
        P2 = A' * P2 * A - (A' * P2 * B2) * inv(R2 + B2' * P2 * B2) * (B2' * P2 * A) + Q2;

        % Compute the feedback gains during each iteration
        F1 = inv(R1 + B1' * P1 * B1) * (B1' * P1 * A);
        F2 = inv(R2 + B2' * P2 * B2) * (B2' * P2 * A);

        % Store feedback gains for plotting
        F1_vals(i, :) = F1;
        F2_vals(i, :) = F2;

        % Print current values of the feedback gains at each iteration:
        fprintf('Iteration %d:\n', i);
        disp('Feedback Gain F1:');
        disp(F1);
        disp('Feedback Gain F2:');
        disp(F2);
    end

    % Plot the convergence of the feedback gains
    figure;
    subplot(2, 1, 1);
    plot(1:num_iterations, F1_vals(:, 1), 'r', 'LineWidth', 2); hold on;
    plot(1:num_iterations, F1_vals(:, 2), 'b', 'LineWidth', 2);
    title('Convergence of Feedback Gain F1');
    legend('F1 - Component 1', 'F1 - Component 2');
    xlabel('Iterations');
    ylabel('Feedback Gain');
    grid on;

    subplot(2, 1, 2);
    plot(1:num_iterations, F2_vals(:, 1), 'r', 'LineWidth', 2); hold on;
    plot(1:num_iterations, F2_vals(:, 2), 'b', 'LineWidth', 2);
    title('Convergence of Feedback Gain F2');
    legend('F2 - Component 1', 'F2 - Component 2');
    xlabel('Iterations');
    ylabel('Feedback Gain');
    grid on;

    % Final Feedback Gains
    fprintf('Final Feedback Gains for Nash Game (F1, F2):\n');
    disp(F1);
    disp(F2);
end
