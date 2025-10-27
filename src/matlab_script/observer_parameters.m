%% Definition of the Observer Parameters 

% (1) Parameters
% Parameters used in the reference paper:
t = 0.04;
a1 = 5; 
a2 = 10; 
a3 = 2;

ts = 0.1;   % Sampling period (s)
S = t/ts;   % Scaling factor
alpha1 = a1 * S;
alpha2 = a2 * S^2;
alpha3 = a3 * S^3;  
% alpha1 = 2, alpha2 = 1.6, alpha3 = 0.128.


% (2) Phi matrix governing the estimation error dynamics
%     of the observer: E(k+1) = Phi * E(k)
Phi = [1 - ts*alpha1, ts,  0;
          -ts*alpha2,  1, ts;
          -ts*alpha3,  0, 1];

% (3) Stability Check of the Observer
eigenvalues = eig(Phi);               % Compute eigenvalues
max_modulus = max(abs(eigenvalues));  % Compute spectral radius

fprintf('max_modulus: %f\n', max_modulus);
if max_modulus < 1
    fprintf('The observer is stable (p(Phi) < 1).\n');
else
    fprintf('WARNING: The observer is NOT stable (p(Phi) >= 1).\n');
end

% RESULT: max_modulus = 0.991042


