%% Computation of Phi_0 via LMI to define the safety margins for CBFs
% Safety distance r_safe = d_min + Ro + rd
% where "rd" is the additional safety margin compensating
% for the observer estimation errors =>
% => (maximum estimation error) rd = Phi_0 * delta 

% Where:

% delta -> Known upper bound of ||E(0)||,
%          where E(0) is the initial estimation error.

% Phi_0 -> Coefficient scaling the initial estimation uncertainty.
%          Used to define a conservative safety margin (rd),
%          derived from the stability analysis of the observer.

% Select eta such that p(Phi) < eta < 1
eta = 0.9999;

% Initialize CVX to find the matrix W.
cvx_begin sdp

    % Decision variable W (symmetric)
    variable W(3, 3) symmetric

    % Constraints
    W >= 1e-6 * eye(3);               % W must be positive definite with eigenvalues ≥ 1e-6
    Phi' * W * Phi - (eta^2) * W <= 0; % LMI inequality

cvx_end

% Check and display results
if cvx_status == 'Solved'
    disp('W found:');
    disp(W);
else
    disp('Error while solving the LMI.');
    disp(cvx_status);
end

% Verify if all eigenvalues of W are positive
eigenvalues_W = eig(W);
disp('Eigenvalues of W:');
    disp(eigenvalues_W);
    if all(eigenvalues_W > 0)
        fprintf('All eigenvalues of W are positive.\n');
    else
        fprintf('WARNING: Not all eigenvalues of W are positive!\n');
    end

LMI_check = Phi' * W * Phi - (eta^2) * W;
% Verify if all eigenvalues of LMI_check are negative
eigenvalues_LMI_check = eig(LMI_check);
disp('Eigenvalues of (Phi^T * W * Phi - eta^2 * W):');
    disp(eigenvalues_LMI_check);
    if all(eigenvalues_LMI_check <= 1e-9)
        fprintf('LMI is satisfied (all eigenvalues <= 0).\n');
    else
        fprintf('WARNING: LMI may not be satisfied (some eigenvalues > 0).\n');
    end

% Compute c1 and c2 (minimum and maximum eigenvalues of W)
c1 = min(eigenvalues_W);
c2 = max(eigenvalues_W); 
fprintf('c1 (lambda_min(W)) = %f\n', c1);
fprintf('c2 (lambda_max(W)) = %f\n', c2);

% Compute Phi_0
Phi_0 = sqrt(c2 / c1);
fprintf('Phi_0 (sqrt(c2/c1)) computed = %f\n', Phi_0);

% RESULT: c1 = 3.386523 , c2 = 19.977028 , Phi_0 = 2.428781