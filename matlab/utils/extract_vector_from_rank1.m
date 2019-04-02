function q = extract_vector_from_rank1(Z)
%extract_vector_from_rank1 ADAPTED FROM JESUS BRIALES' code: 
%                  https://github.com/jbriales/CVPR17

[U,S,V] = svd(Z);
s = diag(S);
% Check that smallest eig is small enough (i.e. numerical error, should be
% zero)
% zero_check = s(end)<1e-3 && (s(end)/s(end-1))<1e-3;

q = sqrt(S)*V.';
q = q(1,:);

% if zero_check
%   % Recovering the solution is trivial, just scale the eigenvector
%   q = U(:,end); %makenonhom(U(:,end));
% else
%   q = U(:,end); %makenonhom(U(:,end));
%   warning('No solution for non-tight case through nullspace yet')
% end

end

function nonhom_x = makenonhom(x)
% nonhom_x = makenonhom( x )
% Returns the homogeneous vector (append 1 at the end of a column vector)

if size(x,2) ~= 1
  error('Use only column vectors with makenonhom');
end

if abs(x(end)) < 1e-6
  warning('Hom component is zero');
end

nonhom_x = x(1:end-1) / x(end);

end