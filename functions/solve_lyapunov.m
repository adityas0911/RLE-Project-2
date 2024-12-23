function F = solve_lyapunov(A, B1, ~, Q, R1, R2, num_iterations) % Solves Lyapunov iterations for Zero-Sum Game.
    gamma = 1; % Zero-sum parameter.
    F = zeros(1, size(A, 2)); % Initial feedback gain.
    tol = 1e-8; % Tighten convergence tolerance.

    for k = 1:num_iterations
        % Solve Lyapunov equation for current feedback gain:

        P = lyap((A - B1 * F)', Q + F' * R1 * F - gamma * (Q + F' * R2 * F));
        F_new = inv(R1 + gamma * R2) * B1' * P;

        % Convergence check:

        if norm(F_new - F) < tol
            disp(['Lyapunov Iterations Converged at Iteration ', num2str(k)]);

            break;
        end

        F = F_new;

        fprintf('Iteration %d: F = [%f %f]\n', k, F);
    end
end
