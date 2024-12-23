function simulate_and_visualize(A, B1, B2, F1, F2, F_lyapunov, x0, time_span) % Simulates and visualizes state trajectories with Riccati and Lyapunov feedback.
    % Closed-loop dynamics:

    A_cl_nash = A - B1 * F1 - B2 * F2; % Nash feedback.
    A_cl_zero_sum = A - B1 * F_lyapunov - B2 * F_lyapunov; % Zero-sum feedback.

    % Simulate trajectories:

    x_nash = zeros(length(x0), length(time_span));
    x_zero_sum = zeros(length(x0), length(time_span));

    for i = 1:length(time_span)
        x_nash(:, i) = expm(A_cl_nash * time_span(i)) * x0;
        x_zero_sum(:, i) = expm(A_cl_zero_sum * time_span(i)) * x0;
    end

    % Create animated visualization (GIF) for Nash Game:

    figure;
    hold on;

    for i = 1:length(time_span)
        plot3(x_nash(1, 1:i), x_nash(2, 1:i), time_span(1:i), 'r-', 'LineWidth', 1.5);
        scatter3(x_nash(1, i), x_nash(2, i), time_span(i), 50, 'filled');
        title('Nash Game: Drone State Trajectories');
        xlabel('Position Drone 1'); ylabel('Position Drone 2'); zlabel('Time');
        grid on;
        pause(0.01);
    end

    % Create animated visualization (GIF) for Zero-Sum Game:

    figure;
    hold on;

    for i = 1:length(time_span)
        plot3(x_zero_sum(1, 1:i), x_zero_sum(2, 1:i), time_span(1:i), 'b-', 'LineWidth', 1.5);
        scatter3(x_zero_sum(1, i), x_zero_sum(2, i), time_span(i), 50, 'filled');
        title('Zero-Sum Game: Drone State Trajectories');
        xlabel('Position Drone 1'); ylabel('Position Drone 2'); zlabel('Time');
        grid on;
        pause(0.01);
    end
end
