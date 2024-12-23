function simulate_trajectories(A, B1, B2, F1, F2, F_lyapunov, x0, time_span)
    % Closed-loop dynamics using feedback:
    A_cl1 = A - B1 * F1 - B2 * F2; 
    A_cl2 = A - B1 * F_lyapunov - B2 * F_lyapunov; % Closed-loop dynamics using Lyapunov feedback
    x_trajectory1 = zeros(length(x0), length(time_span));
    x_trajectory2 = zeros(length(x0), length(time_span));
    
    % Numerical integration using matrix exponentiation:
    for i = 1:length(time_span)
        x_trajectory1(:, i) = expm(A_cl1 * time_span(i)) * x0;
        x_trajectory2(:, i) = expm(A_cl2 * time_span(i)) * x0;
    end
    
    % Plot results: Separate and together for comparison:
    figure;

    subplot(2, 1, 1);
    plot(time_span, x_trajectory1(1, :), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(time_span, x_trajectory1(2, :), 'b-', 'LineWidth', 1.5);
    title('State Trajectories with Riccati Feedback');
    xlabel('Time');
    ylabel('State Variables');
    legend('x1', 'x2');
    grid on;

    subplot(2, 1, 2);
    plot(time_span, x_trajectory2(1, :), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(time_span, x_trajectory2(2, :), 'b-', 'LineWidth', 1.5);
    title('State Trajectories with Lyapunov Feedback');
    xlabel('Time');
    ylabel('State Variables');
    legend('x1', 'x2');
    grid on;

    % Combined plot:
    figure;
    plot(time_span, x_trajectory1(1, :), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(time_span, x_trajectory1(2, :), 'b-', 'LineWidth', 1.5);
    plot(time_span, x_trajectory2(1, :), 'r--', 'LineWidth', 1.5);
    plot(time_span, x_trajectory2(2, :), 'b--', 'LineWidth', 1.5);
    title('State Trajectories: Riccati vs Lyapunov Feedback');
    xlabel('Time');
    ylabel('State Variables');
    legend('x1 (Riccati)', 'x2 (Riccati)', 'x1 (Lyapunov)', 'x2 (Lyapunov)');
    grid on;
end
