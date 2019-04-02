function [abs_error, mean_norm_error] = get_abs_error_quat(xsol, qs)
%get_abs_error 

n = round(length(xsol)/4);
abs_error = 0;
abs_error_norm = 0;
for idx=1:n
    e1 = norm(xsol(4*idx-3:4*idx) - qs(4*idx-3:4*idx))^2;
    e2 = norm(xsol(4*idx-3:4*idx) + qs(4*idx-3:4*idx))^2;
    abs_error = abs_error + min(e1, e2);
    abs_error_norm = abs_error_norm + sqrt(min(e1, e2));
end
mean_norm_error = abs_error_norm/n;

end