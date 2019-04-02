%
% Copyright © 2012, The Massachusetts Institute of Technology. All rights reserved. 
%
% THE LICENSOR EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS 
% SOFIWARE AND DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR ANY PARTICULAR PURPOSE, NON- INFRINGEMENT AND WARRANTIES OF 
% PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM COURSE OF 
% DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH 
% RESPECT TO THE USE OF THE SOFIWARE OR DOCUMENTATION. Under no circumstances 
% shall the Licensor be liable for incidental, special, indirect, direct or 
% consequential damages, or loss of profits, interruption of business, or 
% related expenses which may arise from use of Software or Documentation, 
% including but not limited to those resulting from defects in Software 
% and/or Documentation, or loss or inaccuracy of data of any kind. 
%
% This software is licensed under the "LIMITED RESEARCH LICENSE (SOURCE
% CODE)" as described in the included LICENSE.txt
%
% Please cite the paper below if you are using this software in your work:
% Brookshire, J.; Teller, S. Extrinsic Calibration from Per-Sensor Egomotion. 
%   Robotics: Science and Systems, 2012.
%
function q = bot_matrix_to_quat(R)

    q = zeros(4,1);
    
    m00 = R(1,1); m01 = R(1,2); m02 = R(1,3);
    m10 = R(2,1); m11 = R(2,2); m12 = R(2,3);
    m20 = R(3,1); m21 = R(3,2); m22 = R(3,3);
    
    
    q(1) = sqrt( max( 0, 1 + m00 + m11 + m22 ) ) / 2;
    q(2) = sqrt( max( 0, 1 + m00 - m11 - m22 ) ) / 2;
    q(3) = sqrt( max( 0, 1 - m00 + m11 - m22 ) ) / 2;
    q(4) = sqrt( max( 0, 1 - m00 - m11 + m22 ) ) / 2;
    q(2) = copysign( q(2), m21 - m12 );
    q(3) = copysign( q(3), m02 - m20 );
    q(4) = copysign( q(4), m10 - m01 );

function a = copysign(a,b)
    a = sign(b) * abs(a);