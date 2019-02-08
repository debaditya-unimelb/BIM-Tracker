function dR_Kappa = derivativeR_Kappa(omega,phi,kappa)
% Calculates the derivatives of the 3D rotation matrix R with respect to Kappa.
% Syntax:  
%   dR_Kappa = derivativeR_Kappa(omega,phi,kappa)
% where:
%   omega, phi, kappa are 3 rotation angles in radians and dR_Kappa is a
%   3x3 matrix containnig the derivatves of R with respect to Kappa. The 
%   derivatives are calculated by using the symbolic toolbox and are just
%   evaluated with omega, phi and kappa. R is consistent with makeR.

dR_Kappa = [...
    -cos(phi)*sin(kappa),   cos(kappa)*cos(omega) - sin(kappa)*sin(omega)*sin(phi), cos(kappa)*sin(omega) + cos(omega)*sin(kappa)*sin(phi); ...
    -cos(kappa)*cos(phi), - cos(omega)*sin(kappa) - cos(kappa)*sin(omega)*sin(phi), cos(kappa)*cos(omega)*sin(phi) - sin(kappa)*sin(omega); ...
                       0,                                                        0,                                                      0];