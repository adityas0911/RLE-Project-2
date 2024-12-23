function simulate_and_visualize(A, B1, B2, F1, F2, F_lyapunov, x0, time_span)
    % Pre-allocate state trajectory matrix:
    x = zeros(4, length(time_span));
    x(:,1) = x0;

    % Pre-allocate for control input trajectories
    u_lyapunov_traj = zeros(1, length(time_span)); % For Lyapunov control input
    u_nash_traj = zeros(2, length(time_span)); % For Nash control inputs (Drone 1 and Drone 2)

    % Loop over time steps and simulate drone dynamics:
    for t = 2:length(time_span)
        % Control input for Zero-Sum Game (using Lyapunov)
        u_lyapunov = -F_lyapunov * x(:,t-1);  % Corrected: use F_lyapunov here
        u_lyapunov_traj(t) = u_lyapunov; % Store control input for Lyapunov
        
        % Control input for Nash Game (using Riccati feedback gains)
        u1 = -F1 * x(:,t-1);
        u2 = -F2 * x(:,t-1);
        u_nash_traj(1, t) = u1; % Store control input for Drone 1 (Nash)
        u_nash_traj(2, t) = u2; % Store control input for Drone 2 (Nash)

        % State update for the next time step (using Euler integration)
        x(:,t) = x(:,t-1) + (A * x(:,t-1) + B1 * u1 + B2 * u2) * (time_span(t) - time_span(t-1));
    end

    % Plot the trajectories for both drones (Position and Velocity):
    figure;
    
    % Position Trajectories Plot:
    subplot(2,1,1);
    plot(time_span, x(1,:), time_span, x(2,:));
    title('Position Trajectories');
    legend('Drone 1', 'Drone 2');
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on;

    % Velocity Trajectories Plot:
    subplot(2,1,2);
    plot(time_span, x(3,:), time_span, x(4,:));
    title('Velocity Trajectories');
    legend('Drone 1', 'Drone 2');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    grid on;

    % Adjust plot limits for better visibility:
    set(gcf, 'Position', [100, 100, 800, 600]); % Set figure size

    % Plot the control input trajectories:
    figure;
    subplot(2,1,1);
    plot(time_span, u_lyapunov_traj, 'r', 'LineWidth', 2);
    title('Lyapunov Control Input Trajectory');
    xlabel('Time (s)');
    ylabel('Control Input');
    grid on;

    subplot(2,1,2);
    plot(time_span, u_nash_traj(1,:), 'b', 'LineWidth', 2);
    hold on;
    plot(time_span, u_nash_traj(2,:), 'g', 'LineWidth', 2);
    title('Nash Control Input Trajectories');
    legend('Drone 1', 'Drone 2');
    xlabel('Time (s)');
    ylabel('Control Input');
    grid on;

    % Adjust plot size for visibility:
    set(gcf, 'Position', [100, 100, 800, 600]); % Set figure size
end
