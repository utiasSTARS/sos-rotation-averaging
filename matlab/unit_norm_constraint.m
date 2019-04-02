function [c, c_eq] = unit_norm_constraint(x)
    xsq = reshape(x.^2, 4, []).';
    c_eq = sum(xsq, 2) - ones(size(xsq,1), 1);
    c = [];
end
