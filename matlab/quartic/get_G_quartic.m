function [G] = get_G_quartic(N, I)
%get_G_quartic
n_quartic = length(I);
G = cell(n_quartic, 1);
for l = 1:n_quartic
    Il = I{l};
    Gl = sparse(length(Il)+1, 4*N+1);
    kappa_l = length(Il) + 1;
    for idx=1:length(Il)
        Gl(idx, Il(idx)) = 4;
        Gl(idx, end) = -1/(kappa_l);
    end
    Gl(end, end) = 1;
    G{l} = Gl;
end
end

