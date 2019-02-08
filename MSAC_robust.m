function [t_final, angles_final, R_final, stop_run, Q_delX_best,...
dis_best_set, correspondences_reuse]=MSAC_robust(visible_edges, angles,...
t, edgeim, correspondence_final, Q_delX, MinSegmentLength,...
SearchLengthMSAC, SegmentLength, IRx, IRy, IPPM, f, AccurateMode,...
MaxMSACRuns, MSACSampleSize, JumpOutAngle, JumpoutTranslation,...
UseMahalanobis, MahalanobisDistance, PixelConvergenceThr, FastMode)
% rng(randi(10000,1));
%% MSAC for removing outliers
n=1;
actual_run = 0;
stop_run=0;
%% Setting additional runs for higher accuracy
if AccurateMode
    accurate = 36;
else
    accurate =2;
end

%%
while actual_run<=MaxMSACRuns || n<accurate
    actual_run = actual_run+1;
    %random permutation of the points
    p = randperm(length(correspondence_final(:,1)), MSACSampleSize); 
    xy=correspondence_final(p,1:2);
    XYZ=correspondence_final(p,3:5);

%% % remove colliner lines
    if any(svd(xy)<0.5) 
        continue 
    end

%% Estimate pose from 3 points
    [R_ran(:,:,n),t_ran(n,:), angles_ran(n,:)] = EstimatePose(xy, XYZ,...
    [angles(1) angles(2) angles(3)], t, f); 

%% % if empty vector
    if isempty(t_ran(n,:)==t') || isnan(t_ran(n,1))
        continue;
    end

%% % Eliminate pose estimates beyond this thrushold
    if any(abs(angles_ran(n,:)-angles)>JumpOutAngle*(pi/180)) ||...
            any(abs(t'-t_ran(n,:))>JumpoutTranslation) % Jumpout rules
        continue;
    end

%% Mahal outlier rejection based on covariance mat
if UseMahalanobis
    [~,flag] = cholcov(Q_delX(4:6,4:6),0);
    if flag ==0
        D = pdist2(t_ran(n,:), t', 'mahalanobis',Q_delX(4:6,4:6));
        if D>MahalanobisDistance
            continue;
        end
    end
end

%% create correspondences using new pose
correspondence_final_new{n}=create_correspondences(visible_edges,...
    R_ran(:,:,n), t_ran(n,:), edgeim, f, MinSegmentLength,...
    SearchLengthMSAC, SegmentLength, IRx, IRy, IPPM);

%% if bad pose ignore 
if isempty(correspondence_final_new{n})
   continue; 
end

%% Calculating residuals
reproject_ransac{n} = world_to_image(f ,correspondence_final_new{n}...
    (:,3:5),R_ran(:,:,n),t_ran(n,:)); % reproject the 3d coordinates into
                %the image plane using pose derived from the previosu step
displacement{n} = ((correspondence_final_new{n}(:,1:2) -...
    reproject_ransac{n})*IPPM); % displacement in pixels
disp_rms{n}=sqrt(displacement{n}(:,1).^2+displacement{n}(:,2).^2);

%% MSAC
pixel_thrushold_convergence=1.96*std(disp_rms{n});
greater_than_thr= find(disp_rms{n}>pixel_thrushold_convergence);
smaller_than_thr {n}= find(disp_rms{n}<pixel_thrushold_convergence);
a(n)=length(disp_rms{n}) -length(greater_than_thr);
disp_rms{n}(greater_than_thr)=pixel_thrushold_convergence;
disp_rms{n} = disp_rms{n}.^2;
total_dist(n)=sum(disp_rms{n});

%% check confidence
if FastMode
    ss=find(disp_rms{n}<1);
    inlierProbability = (length(ss)/length(correspondence_final_new{n}...
    (:,1)))^MSACSampleSize;
    num  = log10(1 - 0.99);
    den  = log10(1 - inlierProbability);
    N (n)   = int32(ceil(num/den));
    if actual_run>N(n) && total_dist(n) == min(total_dist)
        disp('jumped out')
        break; 
    end
end

n=n+1;

end
if AccurateMode
    disp(['actual_run: ' num2str(actual_run)])
end

%% MSAC index  =  select the best set 
[~,max_index_full]=min(total_dist); % BEST SET
max_index=max_index_full(1); % Select best set out of many

%% Define convergence criteria
s=find(disp_rms{max_index}<PixelConvergenceThr^2);
if length(s)>0.6*length(correspondence_final_new{max_index})
    stop_run=1;
end

% temporal reuse
dis_best_set = sqrt(disp_rms{max_index}(s));
correspondences_reuse = correspondence_final_new{max_index}(s,:); 


%% Return best pose and the uncertainity after pose refinement
xy = correspondence_final_new{max_index}(smaller_than_thr{max_index},1:2);
XYZ = correspondence_final_new{max_index}(smaller_than_thr{max_index},3:5);
[R_final,t_final, angles_final, Q_delX_best] = EstimatePoseKalman...
    (xy, XYZ, [angles(1) angles(2) angles(3)], t, f);

end