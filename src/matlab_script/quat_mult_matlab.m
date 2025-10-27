% Funzione per la moltiplicazione di due quaternioni
% q1, q2 sono assunti essere [qx; qy; qz; qw]
% Calcola q_out = q1 * q2
function q_out = quat_mult_matlab(q1, q2)
    if size(q1,1) ~= 4 || size(q1,2) ~= 1 || size(q2,1) ~= 4 || size(q2,2) ~= 1
        error('I quaternioni q1 e q2 devono essere vettori colonna 4x1');
    end
    x1 = q1(1); y1 = q1(2); z1 = q1(3); w1 = q1(4);
    x2 = q2(1); y2 = q2(2); z2 = q2(3); w2 = q2(4);

    % Formula del prodotto di Hamilton per q_out = q1 * q2
    % (corrispondente alla tua implementazione CasADi)
    qx_out = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    qy_out = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    qz_out = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    qw_out = w1*w2 - x1*x2 - y1*y2 - z1*z2;

    q_out = [qx_out; qy_out; qz_out; qw_out];
end