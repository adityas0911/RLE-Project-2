function F = solve_lyapunov(A, B1, ~, Q1, Q2, R1, R2, num_iterations)
    % Initialize feedback gain:
    gamma = 1;
    F = zeros(1, size(A, 2)); 
    tol = 1e-6; % Convergence tolerance.
    for k = 1:num_iterations
        P = lyap((A - B1 * F)', Q1 + F' * R1 * F - gamma * (Q2 + F' * R2 * F));
        F_new = inv(R1 + gamma * R2) * B1' * P; 
        
        % Check convergence:
        if norm(F_new - F) < tol
            disp(['Lyapunov converged at iteration ', num2str(k)]);
            break; % Stop if gain converges.
        end
        
        F = F_new;
        fprintf('Iteration %d: F = [%f %f]\n', k, F);
    end
end
