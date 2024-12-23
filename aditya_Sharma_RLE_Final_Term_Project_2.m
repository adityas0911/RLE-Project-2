%% Final Term Project: Zero-Sum Differential Games using Policy Iterations.
%%
% Name: Aditya Sharma.
% NetID: as4108
% RUID: 219008361
% Course: 16:332:515:01 Reinforcement Learning for Engineers
% Professor: z.gajic@rutgers.edu
% Due Date: Dec. 23, 2024 @ 11:59 PM EST
%%
clear; clc; close all; % Clear workspace, command window, and close figures.

addpath('functions'); % Add the functions directory to the MATLAB path.

%% SECTION 1: System Setup and Parameters

% System matrices (2D dynamics):
A = [-0.5, 0.2;
     -0.3, -0.5]; % State matrix.
B1 = [1; 0]; % Control matrix for Player 1.
B2 = [0; 1]; % Control matrix for Player 2.

% Weighting matrices:
Q1 = eye(2); % State weighting for Player 1.
Q2 = eye(2); % State weighting for Player 2.
R1 = 1; % Control effort weighting for Player 1.
R2 = 1; % Control effort weighting for Player 2.

% Simulation parameters:
num_iterations = 10; % Increase to more iterations for better convergence.
x0 = [1; 1]; % Initial state.
time_span = linspace(0, 20, 1000); % Time span for simulations.

%% SECTION 2: Riccati Iterations for Feedback Gains

disp('Running Riccati Iterations...');
[F1, F2] = solve_riccati(A, B1, B2, Q1, Q2, R1, R2, num_iterations);

%% SECTION 3: Lyapunov Iterations for Zero-Sum Games

disp('Running Lyapunov Iterations...');
F_lyapunov = solve_lyapunov(A, B1, B2, Q1, Q2, R1, R2, num_iterations);

%% SECTION 4: Simulate and Plot State Trajectories

disp('Simulating State Trajectories...');
simulate_trajectories(A, B1, B2, F1, F2, F_lyapunov, x0, time_span);
