%% Multi-Agent Drone Stabilization: Zero-Sum and Nash Games

% Name: Aditya Sharma.
% NetID: as4108
% RUID: 219008361
% Course: 16:332:515:01 Reinforcement Learning for Engineers
% Professor: z.gajic@rutgers.edu
% Due Date: Dec. 23, 2024 @ 11:59 PM EST

%% SECTION 0: Matlab Setup
 
clear; clc; close all; % Clear workspace, command window, and close figures.

addpath('functions'); % Add the functions directory to the MATLAB path.

%% SECTION 1: System Setup and Parameters

% Define system matrices (state dynamics):

A = [0, 0, 1, 0; % Position 1 depends on Velocity 1.
     0, 0, 0, 1; % Position 2 depends on Velocity 2.
     -0.1, 0.05, -0.5, 0; % Velocity 1 depends on position & velocity.
     0.05, -0.1, 0, -0.5]; % Velocity 2 depends similarly.

% B1 and B2 represent control influence for Drone 1 and Drone 2:

B1 = [0;
      0;
      1;
      0]; % Drone 1 directly affects its velocity.
B2 = [0;
      0;
      0;
      1]; % Drone 2 directly affects its velocity.

% Cost function weights:

Q = eye(4); % Shared state weighting for zero-sum game.

% Separate state weightings for Nash game:

Q1 = Q;
Q2 = Q;

% Control effort penalties for both drones:

R1 = 1;
R2 = 1;

% Simulation parameters:

num_iterations = 20; % Number of iterations for policy iteration.
x0 = [1;
      -1;
      0.5;
      -0.5]; % Initial state: positions & velocities.
time_span = linspace(0, 20, 1000); % Simulate over 20 seconds.

%% SECTION 2: Policy Iterations for Feedback Gains

% Compute feedback gains using Riccati Iterations (Nash Game):

disp('Running Riccati Iterations for Nash Game...');

[F1, F2] = solve_riccati(A, B1, B2, Q1, Q2, R1, R2, num_iterations);

% Compute feedback gains using Lyapunov Iterations (Zero-Sum Game):

disp('Running Lyapunov Iterations for Zero-Sum Game...');

F_lyapunov = solve_lyapunov(A, B1, B2, Q, R1, R2, num_iterations);

%% SECTION 3: Simulate and Visualize State Trajectories

% Simulate state trajectories with Riccati and Lyapunov feedback:

disp('Simulating and Visualizing State Trajectories...');
simulate_and_visualize(A, B1, B2, F1, F2, F_lyapunov, x0, time_span);
disp('Simulation Complete. All Results Visualized.');
