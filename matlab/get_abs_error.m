function [abs_error, mean_norm_error] = get_abs_error(xsol, qs)
%get_abs_error 

n = round(length(xsol)/4);
abs_error = 0;
abs_error_norm = 0;
for idx=1:n
    Rsol = quat_to_matrix(xsol(4*(idx-1)+1:4*(idx-1)+4));
    Rgt = quat_to_matrix(qs(4*(idx-1)+1:4*(idx-1)+4));
    abs_error = abs_error + norm(Rsol - Rgt, 'fro')^2;
    abs_error_norm = abs_error_norm + norm(Rsol - Rgt, 'fro');
end
mean_norm_error = abs_error_norm/n;

end

