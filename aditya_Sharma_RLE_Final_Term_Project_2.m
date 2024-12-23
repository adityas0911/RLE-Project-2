%% Zero-Sum Differential Games: Riccati and Lyapunov Policy Iterations

% Name: Aditya Sharma
% NetID: as4108
% RUID: 219008361
% Course: 16:332:515:01 Reinforcement Learning for Engineers
% Professor: Z. Gajic
% Due Date: Dec. 23, 2024 @ 11:59 PM EST

%%

clc;
clear;
close all;

%% Define the Drone Dynamics

% State-space model: x_dot = Ax + Bu,   z = Cx
% A represents system dynamics, B control input, C performance output
A = [0 1 0 0;
     0 -0.5 2 0;
     0 0 0 1;
     0 0 -3 -0.7];
B1 = [0; 1; 0; 0];  % Control input for Player 1
B2 = [0; 0; 0; 1];  % Control input for Player 2
C1 = [1 0 0 0];      % Performance output for Player 1
C2 = [0 0 1 0];      % Performance output for Player 2

Q1 = C1' * C1;  % Weighting matrix for Player 1
Q2 = C2' * C2;  % Weighting matrix for Player 2
R1 = 1;         % Control cost for Player 1
R2 = 1;         % Control cost for Player 2

% Zero-sum requirement: Players have opposing goals
Q = Q1 - Q2;   % Combined state weighting matrix
R = R1 - R2;   % Combined control weighting matrix

%% Initialization for Policy Iterations

n = size(A, 1);  % Number of states
m1 = size(B1, 2); % Number of controls for Player 1
m2 = size(B2, 2); % Number of controls for Player 2

% Initial stabilizing gains (K1, K2)
K1 = zeros(m1, n);
K2 = zeros(m2, n);

% Convergence tolerance and max iterations
epsilon = 1e-6;
max_iters = 5;

%% Riccati-Based Policy Iteration

disp('Starting Riccati Iterations...');
P = eye(n); % Initial guess for P
performance_riccati = zeros(max_iters, 1);
feedback_gains_riccati = cell(max_iters, 1);

for k = 1:max_iters
    % Compute feedback gains using Riccati equation
    K1 = -inv(R1 + B1' * P * B1) * (B1' * P * A);
    K2 = -inv(R2 + B2' * P * B2) * (B2' * P * A);

    % Store feedback gains
    feedback_gains_riccati{k} = {K1, K2};

    % Compute closed-loop dynamics
    A_cl = A + B1 * K1 + B2 * K2;

    % Solve Riccati equation using a numerical solver
    P_next = Q + A_cl' * P * A_cl;

    % Compute performance metric
    performance_riccati(k) = trace(P_next);

    % Check convergence
    if norm(P_next - P, 'fro') < epsilon
        break;
    end

    P = P_next;
end

%% Lyapunov-Based Policy Iteration

disp('Starting Lyapunov Iterations...');
P = eye(n); % Initial guess for P
performance_lyapunov = zeros(max_iters, 1);
feedback_gains_lyapunov = cell(max_iters, 1);

for k = 1:max_iters
    % Compute feedback gains using Lyapunov equation
    K1 = -inv(R1 + B1' * P * B1) * (B1' * P * A);
    K2 = -inv(R2 + B2' * P * B2) * (B2' * P * A);

    % Store feedback gains
    feedback_gains_lyapunov{k} = {K1, K2};

    % Compute closed-loop dynamics
    A_cl = A + B1 * K1 + B2 * K2;

    % Solve Lyapunov equation using a numerical solver
    P_next = Q + A_cl' * P * A_cl;

    % Compute performance metric
    performance_lyapunov(k) = trace(P_next);

    % Check convergence
    if norm(P_next - P, 'fro') < epsilon
        break;
    end

    P = P_next;
end

%% Simulating State Trajectories

x0 = [1; 0; -1; 0];  % Initial state
T = 0:0.1:10;        % Time vector

% Riccati Trajectories
state_traj_riccati = zeros(length(T), n, max_iters);
for k = 1:max_iters
    K1 = feedback_gains_riccati{k}{1};
    K2 = feedback_gains_riccati{k}{2};
    A_cl = A + B1 * K1 + B2 * K2;

    % Simulate dynamics
    [~, X] = ode45(@(t, x) A_cl * x, T, x0);
    state_traj_riccati(:, :, k) = X;
end

% Lyapunov Trajectories
state_traj_lyapunov = zeros(length(T), n, max_iters);
for k = 1:max_iters
    K1 = feedback_gains_lyapunov{k}{1};
    K2 = feedback_gains_lyapunov{k}{2};
    A_cl = A + B1 * K1 + B2 * K2;

    % Simulate dynamics
    [~, X] = ode45(@(t, x) A_cl * x, T, x0);
    state_traj_lyapunov(:, :, k) = X;
end

%% Plotting Feedback Gains

figure;
for i = 1:max_iters
    K1_riccati = feedback_gains_riccati{i}{1};
    K2_riccati = feedback_gains_riccati{i}{2};
    subplot(2, 1, 1);
    plot(1:size(K1_riccati, 2), K1_riccati, '-o'); hold on;
    title('Riccati Gains K1');
    xlabel('State Index'); ylabel('Gain Value');

    subplot(2, 1, 2);
    plot(1:size(K2_riccati, 2), K2_riccati, '-o'); hold on;
    title('Riccati Gains K2');
    xlabel('State Index'); ylabel('Gain Value');
end

figure;
for i = 1:max_iters
    K1_lyapunov = feedback_gains_lyapunov{i}{1};
    K2_lyapunov = feedback_gains_lyapunov{i}{2};
    subplot(2, 1, 1);
    plot(1:size(K1_lyapunov, 2), K1_lyapunov, '-o'); hold on;
    title('Lyapunov Gains K1');
    xlabel('State Index'); ylabel('Gain Value');

    subplot(2, 1, 2);
    plot(1:size(K2_lyapunov, 2), K2_lyapunov, '-o'); hold on;
    title('Lyapunov Gains K2');
    xlabel('State Index'); ylabel('Gain Value');
end

%% Plotting State Trajectories

figure;
for k = 1:max_iters
    subplot(2, 3, k);
    plot(T, state_traj_riccati(:, :, k));
    title(['Riccati Trajectories (Iter ' num2str(k) ')']);
    xlabel('Time'); ylabel('State');
end

figure;
for k = 1:max_iters
    subplot(2, 3, k);
    plot(T, state_traj_lyapunov(:, :, k));
    title(['Lyapunov Trajectories (Iter ' num2str(k) ')']);
    xlabel('Time'); ylabel('State');
end

%% Performance Tables

table_riccati = table((1:max_iters)', performance_riccati, 'VariableNames', {'Iteration', 'Performance_Riccati'});
table_lyapunov = table((1:max_iters)', performance_lyapunov, 'VariableNames', {'Iteration', 'Performance_Lyapunov'});

disp('Performance Table (Riccati):');
disp(table_riccati);
disp('Performance Table (Lyapunov):');
disp(table_lyapunov);
