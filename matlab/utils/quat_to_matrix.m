function R = quat_to_matrix(quat)
    %quat = quat/norm(quat);
    
    qw=quat(1);
    qx=quat(2);
    qy=quat(3);
    qz=quat(4);
    qx2 = qx^2;
    qy2 = qy^2;
    qz2 = qz^2;
    qw2 = qw^2;
    R = [qw2 + qx2 - qy2 - qz2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw; ...
         2*qx*qy + 2*qz*qw, qw2 - qx2 + qy2 - qz2, 2*qy*qz - 2*qx*qw; ...
         2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, qw2 - qx2 - qy2 + qz2];

end