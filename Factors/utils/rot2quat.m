function quat = rot2quat(rot)
%ROT2QUAT Rodrigues to Quaternion
%   input - rot - The Rodrigues form of a rotation, angle*axis, 3x1
%   output - quat - The Quaternion form of the same rotation, xyzw, 4x1

angle = norm(rot);
axis = rot/angle; 

w = cos(angle/2);
xyz = axis*sin(angle/2);

quat = [xyz; w];

end

