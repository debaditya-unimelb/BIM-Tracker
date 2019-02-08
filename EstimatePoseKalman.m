function [R,t, angles, Q_delX] = EstimatePoseKalman(xy, XYZ,...
    ini_rot, ini_pos, f)
%% Variances for x,y X,Y,Z MRamezani (4/10/2018)
var_XYZ = 50^2; % uncertainity of 3D model that is also dependent 
% on the uncertanity of point clouds used for building the model
var_xy = (0.025)^2; %uncertanity of image plane and considered as 5 pixels
% experimentally found with real data


Q_xyXYZ = [var_xy*eye(2) zeros(2,3); zeros(3,2) var_XYZ*eye(3)];
%% init
nPnts = size(xy, 1);
x = xy(:,1); y = xy(:,2);
X = XYZ(:,1); Y = XYZ(:,2); Z = XYZ(:,3);

% iteration control parameters
itrMax = 20;
thrStop = 1e-5;

% initial values for the unknowns
omega = ini_rot(1); phi = ini_rot(2); kappa = ini_rot(3);
R = makeR3(omega, phi, kappa);
Xc = ini_pos(1); Yc = ini_pos(2); Zc = ini_pos(3);

% matrix allocations
A = zeros(2*nPnts,6);
b = zeros(2*nPnts,1);
res = zeros(nPnts,itrMax);
J_b = zeros(2*nPnts,5);

%% estimate R and t

% Observation equations for one pair of points ([x,y], [X,Y,Z]):
% F1 = (f*r_11 + x*r_31)*(X - Xc) + (f*r_12 + x*r_32)*(Y - Yc) 
% + (f*r_13 + x*r_33)*(Z - Zc) = 0
% F2 = (f*r_21 + y*r_31)*(X - Xc) + (f*r_22 + y*r_32)*(Y - Yc)
% + (f*r_23 + y*r_33)*(Z - Zc) = 0

for itr = 1:itrMax
    for pnt = 1:nPnts
        % Evaluate the equations with initial values for unknowns
        F1o = (f*R(1,1) + x(pnt)*R(3,1))*(X(pnt) - Xc) + (f*R(1,2)...
            + x(pnt)*R(3,2))*(Y(pnt) - Yc) + (f*R(1,3)...
            + x(pnt)*R(3,3))*(Z(pnt) - Zc);
        F2o = (f*R(2,1) + y(pnt)*R(3,1))*(X(pnt) - Xc) + (f*R(2,2)...
            + y(pnt)*R(3,2))*(Y(pnt) - Yc) + (f*R(2,3)...
            + y(pnt)*R(3,3))*(Z(pnt) - Zc);
        
        % calculate derivatives of the observation equation wrt unknowns
        derF1_r11 = f * (X(pnt) - Xc);
        derF1_r12 = f * (Y(pnt) - Yc);
        derF1_r13 = f * (Z(pnt) - Zc);
        derF1_r21 = 0;
        derF1_r22 = 0;
        derF1_r23 = 0;
        derF1_r31 = x(pnt) * (X(pnt) - Xc);
        derF1_r32 = x(pnt) * (Y(pnt) - Yc);
        derF1_r33 = x(pnt) * (Z(pnt) - Zc);
        
        derF2_r11 = 0;
        derF2_r12 = 0;
        derF2_r13 = 0;
        derF2_r21 = f * (X(pnt) - Xc);
        derF2_r22 = f * (Y(pnt) - Yc);
        derF2_r23 = f * (Z(pnt) - Zc);
        derF2_r31 = y(pnt) * (X(pnt) - Xc);
        derF2_r32 = y(pnt) * (Y(pnt) - Yc);
        derF2_r33 = y(pnt) * (Z(pnt) - Zc);
        
        derR_omega = derivativeR_Omega(omega,phi,kappa);
        derR_phi = derivativeR_Phi(omega,phi,kappa);
        derR_kappa = derivativeR_Kappa(omega,phi,kappa);
        
        derF1_rij = [derF1_r11, derF1_r21, derF1_r31, derF1_r12,...
            derF1_r22, derF1_r32, derF1_r13, derF1_r23, derF1_r33];
        derF2_rij = [derF2_r11, derF2_r21, derF2_r31, derF2_r12,...
            derF2_r22, derF2_r32, derF2_r13, derF2_r23, derF2_r33];
        
        derF1_omega = derF1_rij*derR_omega(:);
        derF2_omega = derF2_rij*derR_omega(:);
        derF1_phi = derF1_rij*derR_phi(:);
        derF2_phi = derF2_rij*derR_phi(:);
        derF1_kappa = derF1_rij*derR_kappa(:);
        derF2_kappa = derF2_rij*derR_kappa(:);

        derF1_Xc = -(f*R(1,1) + x(pnt)*R(3,1));
        derF2_Xc = -(f*R(2,1) + y(pnt)*R(3,1));
        derF1_Yc = -(f*R(1,2) + x(pnt)*R(3,2));
        derF2_Yc = -(f*R(2,2) + y(pnt)*R(3,2));
        derF1_Zc = -(f*R(1,3) + x(pnt)*R(3,3));
        derF2_Zc = -(f*R(2,3) + y(pnt)*R(3,3));

        % fill in the coefficient matrix with all derivatives
        A(2*pnt-1,:) = [derF1_omega, derF1_phi, derF1_kappa, derF1_Xc,...
            derF1_Yc, derF1_Zc];
        A(2*pnt-0,:) = [derF2_omega, derF2_phi, derF2_kappa, derF2_Xc,...
            derF2_Yc, derF2_Zc];
        
        % fill in b with the two equations evaluated with initial values
        b(2*pnt-1) = -F1o;
        b(2*pnt-0) = -F2o;
        
        % Calculation of Jacobi_b , MRamezani(4/10/2018)
        derF10_x = R(3,1)*(X(pnt)-Xc) + R(3,2)*(Y(pnt)-Yc) +...
            R(3,3)*(Z(pnt)-Zc);
        derF10_y = 0;
        derF10_X = f*R(1,1) + x(pnt)*R(3,1);
        derF10_Y = f*R(1,2) + x(pnt)*R(3,2);
        derF10_Z = f*R(1,3) + x(pnt)*R(3,3);
        derF20_x = 0;
        derF20_y = R(3,1)*(X(pnt)-Xc) + R(3,2)*(Y(pnt)-Yc) +...
            R(3,3)*(Z(pnt)-Zc);
        derF20_X = f*R(2,1) + y(pnt)*R(3,1);
        derF20_Y = f*R(2,2) + y(pnt)*R(3,2);
        derF20_Z = f*R(2,3) + y(pnt)*R(3,3);
        
        J_b(2*pnt-1,:) = [derF10_x derF10_y derF10_X derF10_Y derF10_Z];
        J_b(2*pnt-0,:) = [derF20_x derF20_y derF20_X derF20_Y derF20_Z];
        
    end
    
    % Calculate the Q_b MRamezani (4/10/2018)
    Q_b = J_b * Q_xyXYZ * J_b';
    
    J_delX = (A'*A)\A';
    % estimate corrections to unknowns
    deltaX = J_delX*b;
    
    % Calculate the Q_delX MRamezani (4/10/2018)

    Q_delX = J_delX * Q_b * J_delX';
    
    % update unknowns
    omega = omega + deltaX(1); phi = phi + deltaX(2); kappa = kappa...
        + deltaX(3);
    angles = [omega phi kappa];
    R = makeR3(omega, phi, kappa);
    Xc = Xc + deltaX(4); Yc = Yc + deltaX(5); Zc = Zc + deltaX(6);
    t = [Xc, Yc, Zc]';
    
    % calculate residuals
    vx = x + f*(R(1,1)*(X-Xc)+R(1,2)*(Y-Yc)+R(1,3)*(Z-Zc)) ./...
        (R(3,1)*(X-Xc)+R(3,2)*(Y-Yc)+R(3,3)*(Z-Zc));
    vy = y + f*(R(2,1)*(X-Xc)+R(2,2)*(Y-Yc)+R(2,3)*(Z-Zc)) ./...
        (R(3,1)*(X-Xc)+R(3,2)*(Y-Yc)+R(3,3)*(Z-Zc));
    res(:,itr) = sqrt(vx.^2 + vy.^2);
    
    % if residuals or corrections are too small stop the iteration
    if ((sum(res(:,itr))/nPnts < thrStop) || all(abs(deltaX) < thrStop))
        res = res(:,1:itr);
        break
    end    
end

