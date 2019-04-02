function R = random_rot_matrix(d, phi_max)
%random_rot_matrix Generate a random matrix with axis-angle, distribution
%unknown, hopefully uniform but not important.

if nargin < 2
    phi_max = pi;
end
a = rand(d, 1);
a = a/norm(a,2);
t = rand()*2*phi_max - phi_max;

%% TODO: compare with axis_angle.m
c = cos(t);
s = sin(t);
C = 1-c;
x = a(1);
y = a(2);
z = a(3);

R = [x*x*C+c x*y*C-z*s x*z*C+y*s;
     y*x*C+z*s y*y*C+c y*z*C-x*s;
     z*x*C-y*s z*y*C+x*s z*z*C+c];
end

