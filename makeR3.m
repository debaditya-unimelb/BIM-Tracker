function R = makeR3(rx, ry, rz)

% Makes a 3d rotation matrix from 3 rotation angles
% Syntax:
%     R = makeR3(rx, ry, rz)
% where:
%     rx, ry, rz are rotation angles in degrees around the three axes and
%     R is a 3x3 rotation matrix.
 
Rx = [1     0       0;
      0  cos(rx) sin(rx);
      0 -sin(rx) cos(rx)];

Ry = [cos(ry) 0 -sin(ry);
         0     1    0;
      sin(ry) 0  cos(ry)];
  
Rz = [cos(rz) sin(rz) 0;
     -sin(rz) cos(rz) 0;
          0       0     1];

R = Rz*Ry*Rx;
