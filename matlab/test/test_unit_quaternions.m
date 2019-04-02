function [is_unit] = test_unit_quaternions(qs, constr_tol)
%test_unit_quaternions Ensure qs is composed of unit quaternions
if nargin < 2
    constr_tol = 1e-6;
end
N = length(qs);
assert(mod(N, 4) == 0, 'Need 4n elements');
[~, ceq] = unit_norm_constraint(qs);
is_unit = all(abs(ceq) <= constr_tol);
end