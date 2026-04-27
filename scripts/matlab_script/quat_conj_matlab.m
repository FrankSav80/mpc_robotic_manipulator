% Funzione per il coniugato di un quaternione
% q è assunto essere [qx; qy; qz; qw]
function q_conj = quat_conj_matlab(q)
    if size(q,1) ~= 4 || size(q,2) ~= 1
        error('Il quaternione q deve essere un vettore colonna 4x1');
    end
    q_conj = [-q(1); -q(2); -q(3); q(4)];
end