function [F1, F2] = solve_riccati(A, B1, B2, Q1, Q2, R1, R2, num_iterations)
    % Initialize feedback gains:
    F1 = zeros(1, size(A, 2)); 
    F2 = zeros(1, size(A, 2));
    tol = 1e-6; % Convergence tolerance.
    for k = 1:num_iterations
        P1 = care(A - B2 * F2, B1, Q1, R1);
        P2 = care(A - B1 * F1, B2, Q2, R2);
        
        % Update feedback gains:
        F1_new = inv(R1) * B1' * P1;
        F2_new = inv(R2) * B2' * P2;
        
        % Check convergence:
        if norm(F1_new - F1) < tol && norm(F2_new - F2) < tol
            disp(['Riccati converged at iteration ', num2str(k)]);
            break; % Stop if gains converge.
        end
        
        F1 = F1_new;
        F2 = F2_new;
        fprintf('Iteration %d: F1 = [%f %f], F2 = [%f %f]\n', k, F1, F2);
    end
end
