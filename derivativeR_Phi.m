function dR_Phi = derivativeR_Phi(omega,phi,kappa)
% Calculates the derivatives of the 3D rotation matrix R with respect to Phi.
% Syntax:  
%   dR_Phi = derivativeR_Phi(omega,phi,kappa)
% where:
%   omega, phi, kappa are 3 rotation angles in radians and dR_Phi is a
%   3x3 matrix containnig the derivatves of R with respect to phi. The 
%   derivatives are calculated by using the symbolic toolbox and are just
%   evaluated with omega, phi and kappa. R is consistent with makeR.

dR_Phi = [...
    -cos(kappa)*sin(phi),  cos(kappa)*cos(phi)*sin(omega), -cos(kappa)*cos(omega)*cos(phi); ...
     sin(kappa)*sin(phi), -cos(phi)*sin(kappa)*sin(omega),  cos(omega)*cos(phi)*sin(kappa); ...
                cos(phi),             sin(omega)*sin(phi),            -cos(omega)*sin(phi)];