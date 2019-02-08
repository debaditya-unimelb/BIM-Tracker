function dR_Omega = derivativeR_Omega(omega,phi,kappa)
% Calculates the derivatives of the 3D rotation matrix R with respect to Omega.
% Syntax:  
%   dR_Omega = derivativeR_Omega(omega,phi,kappa)
% where:
%   omega, phi, kappa are 3 rotation angles in radians and dR_Omega is a
%   3x3 matrix containnig the derivatves of R with respect to omega. The 
%   derivatives are calculated by using the symbolic toolbox and are just
%   evaluated with omega, phi and kappa. R is consistent with makeR.

dR_Omega = [...
    0,   cos(kappa)*cos(omega)*sin(phi) - sin(kappa)*sin(omega), cos(omega)*sin(kappa) + cos(kappa)*sin(omega)*sin(phi); ...
    0, - cos(kappa)*sin(omega) - cos(omega)*sin(kappa)*sin(phi), cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi); ...
    0,                                     -cos(omega)*cos(phi),                                   -cos(phi)*sin(omega)];