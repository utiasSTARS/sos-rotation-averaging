function [F] = get_F_mle(R_meas, E, w)
N = max(max(E));
M = size(R_meas, 3);
assert(M == size(E,1));
if nargin < 3
    w = ones(M, 1);
end

qi = sym('qi', [4 1]);
qj = sym('qj', [4 1]);
Rij = sym('Rij', [3 3]);
Ri = quat_to_matrix(qi);
Rj = quat_to_matrix(qj);
arg_norm = Ri*Rij - Rj;

f = simplify(sum(sum(arg_norm.^2)));
inds = get_poly_info();

%% Monomial size
% There are 36 monomials per term in the cost function
% There are 4*N variables plus 1 term for the coefficient
%F = sparse(36*M, 4*N + 1);
I = [];
J = [];
V = [];
for m=1:M
    Rij_m = R_meas(:,:,m); 
    f_m = subs(f, Rij(:), Rij_m(:));
    ind1 = E(m,1);
    ind2 = E(m,2);
%     F_ind = 1;
    [Im, Jm, Vm] = quat_matrix_mapping(inds, f_m, 4*(ind1-1), 4*(ind2-1), qi, qj, 4*N+1);
    if ~isempty(I)
        I = [I Im+I(end)];
    else
        I = [I Im];
    end
    J = [J Jm];
    V = [V w(m)^2*double(Vm)];
end

F = sparse(I,J,V);

end

function [I, J, V] = quat_matrix_mapping(inds, f, q1_ind, q2_ind, q1, q2, m_max)
I = [];
J = [];
V = [];
q = [q1; q2];
I_count = 1;

[coeff_list, term_list] = coeffs(f, [q1; q2]);
for k=1:size(inds, 1)    
    poly_ind = inds(k,:);
    var_k = 1;
    for l=1:size(poly_ind,2)
        if poly_ind(l) > 0
            var_k = var_k*q(l)^poly_ind(l);
        end
    end
    %% Need to use find(t == var_k) on full coeffs output!
    %coeff_kl = coeffs(f, var_k);
    coeff_index = find(term_list == var_k);
    if ~isempty(coeff_index)%  isnumeric(coeff_kl) && abs(coeff_kl) > 0
        coeff_kl = coeff_list(coeff_index);
        % Empty it out for debugging - also, there should probably be no
        % double ups??
        coeff_list(coeff_index) = [];
        term_list(coeff_index) = [];
        for l=1:size(poly_ind,2)
            if poly_ind(l) > 0
                I = [I I_count];
                if l <= 4
                    J = [J l+q1_ind];
                else
                    J = [J l-4+q2_ind];
                end
                V = [V poly_ind(l)];
            end
        end
        I = [I I_count];
        J = [J m_max];
        I_count = I_count + 1;
        V = [V coeff_kl];
    end
end

end


function [inds] = get_poly_info()
% d - degree of polynomial
% k - number of vars

if exist('quartic_poly_inds.mat', 'file')
    load('quartic_poly_inds.mat');
else
    p1 = unique(perms([1 1 1 1 0 0 0 0]), 'rows');
    p2 = unique(perms([2 1 1 0 0 0 0 0]), 'rows');
    p22 = unique(perms([2 2 0 0 0 0 0 0]), 'rows');
    % p3 = perms([3 1 0 0 0 0 0 0]);
    p4 = unique(perms([4 0 0 0 0 0 0 0]), 'rows');
    inds = [p1; p2; p22; p4];
    save('quartic_poly_inds.mat', 'inds');
end

end