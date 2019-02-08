%% Load data
clear;
close all;
load('aux_data_real.mat'); %load edge images and visible model edges
warning('off','all')

%% SETTINGS

% camera parameters
f = 3.3895; % focal length camera
IPPM = 176.4706; % camera scale factor
IRx = 640; % camera X resolution
IRy = 480; % camera Y resolution

%MSAC parameters
MaxMSACRuns = 500; % max MSAC iteration, should not be less than 293
MinSegmentLength = 0; % eliminate small lines
SegmentLength = 10; % number of pixels to segment each line 
SearchLength = 100; % search each side of back-projected line
SearchLengthMSAC = 50; % search for back-projected line within loop
CorrespondenceUpdates = 5;% number of correspondence updates
MSACSampleSize = 3; % sample size for performing pose estimation using MSAC
JumpOutAngle = 5; % Jumpout angle
JumpoutTranslation = 500; % jump out translation
PixelConvergenceThr = 3;% convergence thrushold 

% Mahalanobis outlier parameters
UseMahalanobis = true; % use constraints from Mahalanobis distance
MahalanobisDistance = 5; % number of standard davations for Mahalanobis

%reusing data from previous frame
ReuseData = true; % use data from previous frames
ReuseFactor = 0.4; % Thrushold for ratio of inliers/total for reuse
ReusePixelThr = 5; % Thrushold for of inliers for reuse

% Select modes. DO NOT use together!
AccurateMode = false; % increase the ransac samples to guarantee solution
                      % will override maximum msac runs settings
                      
FastMode = false; % jump out loop if required confidence reached and lower
                 % loop iterations
if FastMode
    MaxMSACRuns = 300;
end

% visualisation data online
visualiseFrame = true; % online frame visualisation, may impact speed
visualiseTrajectory = true; % trajectory visualisation, may impact speed

%% Provide initilisation
ang_kalman=[-19.119,1.453,20.675];
t_kalman=[3450.325,9766.037,976.889]';

% Kalman parameters
Ts=1/30;

%define the state matrix (F) for tranlation
At=[1 0 0 Ts 0 0 0 0 0 0 0 0;
    0 1 0 0 Ts 0 0 0 0 0 0 0;
    0 0 1 0 0 Ts 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 Ts 0 0;
    0 0 0 0 0 0 0 1 0 0 Ts 0;
    0 0 0 0 0 0 0 0 1 0 0 Ts;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1]; 

%define the output matrix (H)
Ct=[1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0]; 

%define the initial state conditions
xt0=[-19.064;1.445;20.661;0.0175;0.0175;0.0175;...
    3444.7451;9734.074;866.112;-600;30;30]; 

% first state of Kalman filter
Xtest(:,1)=xt0; 

%PSD*Ts of rotational and tranlational accelaration, found experimentally 
qqav = 0.05; 
qqav2 = 0.005; 
qqtv = 5; 

%covariance matrix for process noise
Qt=[0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 (qqav).^2 0 0 0 0 0 0 0 0;
    0 0 0 0 (qqav2).^2 0 0 0 0 0 0 0;
    0 0 0 0 0 (qqav).^2 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 (qqtv).^2 0 0;
    0 0 0 0 0 0 0 0 0 0 (qqtv).^2 0;
    0 0 0 0 0 0 0 0 0 0 0 (qqtv).^2]; 

%inital value of covarience of estimation error
Pt=1*eye(12,12); 

% Initial pose covariance matrix for Guass-Newton minimisation 
Q_delX{1} = [1e-3 0 0 0 0 0;
            0 1e-6 0 0 0 0; 
            0 0 1e-3 0 0 0;
            0 0 0 2500 0 0;
            0 0 0 0 2500 0;
            0 0 0 0 0 2500]; 
                             
% reusing params for the first frame
correspondences_reuse = [];
dis_best_set = [];
        
%% Run main loop for each image
for frame =1:550
   % to measure time
    tic 
    
   % to assign visible edges of each frame from the stack.
        visible_edges_frame=visible_edges_all_frames{frame}; 

%% predicted state, correct correspondences and covariance from MSAC

    [t_est, ang_est, itr(frame), Q_delX{frame+1}, reuse(frame),...
    correspondences_reuse, dis_best_set] = MSAC(visible_edges_frame,...
    ang_kalman, t_kalman, frame, edgeim(:,:,frame), edgeim(:,:,frame+1),...
    correspondences_reuse, dis_best_set, Q_delX{frame},...
    MinSegmentLength, SearchLength, SearchLengthMSAC, SegmentLength,...
    IRx, IRy, IPPM, f,AccurateMode, MaxMSACRuns, MSACSampleSize,...
    JumpOutAngle,JumpoutTranslation, UseMahalanobis,...
    MahalanobisDistance, PixelConvergenceThr, FastMode, ReusePixelThr,...
    ReuseFactor, CorrespondenceUpdates, ReuseData);


%% Kalman filtering
    
    % Covariance from Newton-Guass minimisation
    measurmentsVt= Q_delX{frame+1}; %covariance for measurement noise
    Rt=measurmentsVt*(Ct*Ct');
    
    % predictions
    Pt=At*Pt*At'+Qt; %predicting P 
    Xtest(:,frame+1)=At*Xtest(:,frame); %Predicitng the state 
    Kt=Pt*Ct'/(Ct*Pt*Ct'+Rt); %calculating the Kalman gain
    
    %Updating: estimating the state
    Xtest(:,frame+1)=Xtest(:,frame+1)+Kt*([ang_est(1); ang_est(2);...
    ang_est(3); t_est(1); t_est(2); t_est(3);]-Ct*Xtest(:,frame+1)); 
    Pt=(eye(12)-Kt*Ct)*Pt ; %Correcting: estimating P 

%% Initialisation for next frame
    ang_kalman = ang_est;
    t_kalman = t_est;    
    
%% Keep a stack of all variables
    t_est_stack(frame,:)=t_est;
    ang_est_stack(frame,:)=ang_est;
    t_est_stack_kalman(frame,:) = Xtest(7:9,frame+1);
    ang_est_stack_kalman(frame,:) = Xtest(1:3,frame+1);
    
%% Visualise the frame with pose
if visualiseFrame
    R = makeR3(ang_kalman(1), ang_kalman(2), ang_kalman(3));
    figure(1)
    imshow(edgeim(:,:,frame))
    title(['Matching at frame:' num2str(frame)])
    hold on
    A=world_to_pixel(f,...
        visible_edges_frame(:,2:4), R, t_kalman,...
        IRx, IRy, IPPM);
    B=world_to_pixel(f,...
        visible_edges_frame(:,5:7), R, t_kalman,...
        IRx, IRy, IPPM);
    if reuse(frame)
        A_reuse = world_to_pixel(f,...
        visible_edges_frame(:,2:4),R, t_kalman,...
        IRx, IRy, IPPM);
        B_reuse = world_to_pixel(f,...
        visible_edges_frame(:,5:7),R, t_kalman,...
        IRx, IRy, IPPM);
        for nn =1:length(A_reuse)
            plot([A_reuse(nn,1); B_reuse(nn,1)], [A_reuse(nn,2);...
                B_reuse(nn,2)], 'b', 'LineWidth', 5)
        end
    else
        for nn =1:length(A)
            plot([A(nn,1); B(nn,1)], [A(nn,2); B(nn,2)], 'g')
        end
    end
    hold off
end

%% Trajectory visualisation
if visualiseTrajectory
    if frame>1
        figure(2)
        hold on
        line(t_est_stack_kalman(frame-1:frame,1),...
        t_est_stack_kalman(frame-1:frame,2),...
        t_est_stack_kalman(frame-1:frame,3), 'Color', 'r')
        line(t_est_stack(frame-1:frame,1),...
        t_est_stack(frame-1:frame,2),t_est_stack(frame-1:frame,3),...
        'Color', 'g')
        legend('Kalman','Estimation')
        axis equal
    end
end

%%
pause(0.0001) % refresh figure
time(frame)=toc;
disp(['frame no. ' num2str(frame)])
end


%% Plotting final trajectorys
figure
hold on
line(t_est_stack_kalman(:,1), t_est_stack_kalman(:,2),...
t_est_stack_kalman(:,3), 'Color', 'r')
line(t_est_stack(:,1), t_est_stack(:,2),t_est_stack(:,3), 'Color', 'g')
legend('Kalman','Estimation')
axis equal

%% Statistics
disp(['Average time taken per frame: ' num2str(mean(time))])
disp(['Ratio of data reuse: ' num2str(mean(reuse))])
