function [correspondence_final_unique]=create_correspondences(visible_edges, R, t, edgeim, f, minlinelength, search_len, seglinelength, IRx, IRy, IPPM)
%% FUNCTION FOR FINDING SINGLE HYPOTHESIS FOR EACH SAMPLED POINTS - UNOPTIMISED VERSION
A=[];
B=[];
correspondence_final=[];
test_final=[];
A_old=world_to_pixel(f,visible_edges(:,2:4),R,t, IRx, IRy, IPPM);
B_old=world_to_pixel(f,visible_edges(:,5:7),R,t, IRx, IRy, IPPM);

%% CALCULATING SLOPE, NUMBER OF SEGMENTS, SEGMENTED 2D COORDINATES, SEGMENTED 3D COORDINATES and REMOVING SMALL LINES FROM THE MODEL 
k=1;
h=1;
for n =1:length(visible_edges)
%% Calculating few things
    F(n) = sqrt((A_old(n,1) - B_old(n,1))^2 + (A_old(n,2) - B_old(n,2))^2);
    centroid_visible_edges=(visible_edges(n,2:4)+visible_edges(n,5:7))/2;
    cam_dist(n)=sqrt((centroid_visible_edges(1)-t(1))^2+ (centroid_visible_edges(2)-t(2))^2+(centroid_visible_edges(3)-t(3))^2);
    
%% deleting small model lines
    if F(n)>minlinelength %&& cam_dist(n)<7000
        A(k,:)=A_old(n,:);B(k,:)=B_old(n,:);
        slopes(k)=(B(k,2)-A(k,2))/(B(k,1)-A(k,1));
        segments=floor(F(n)/ seglinelength)+1;
        visible_edges_reduced(k,:)=visible_edges(n,2:7);
        
        if segments < 10
            n_points(k) = segments;
        else
            n_points(k) = 10;
        end
        
%% sampling model lines in 2d space
        samp_points_2d=[];
        samp_points_3d=[];
        matches=[];
        
        for m=1:n_points(k)+1
            samp_points_3d(m,:)=[(visible_edges_reduced(k,1) - (m-1)*(visible_edges_reduced(k,1)-visible_edges_reduced(k,4))/n_points(k))...
                (visible_edges_reduced(k,2) - (m-1)*(visible_edges_reduced(k,2)-visible_edges_reduced(k,5))/n_points(k))...
                (visible_edges_reduced(k,3) - (m-1)*(visible_edges_reduced(k,3)-visible_edges_reduced(k,6))/n_points(k))];
            samp_points_2d(m,:)=world_to_pixel(f,samp_points_3d(m,:) ,R,t, IRx, IRy, IPPM);
            
            % calculting pixels perpendicular to sampled points at a certain distance
           
            if samp_points_2d(m,1)<IRx+1 && samp_points_2d(m,1)>0 && samp_points_2d(m,2)<IRy+1 && samp_points_2d(m,2)>0
            for len=0:search_len-1
%                 [x1, y1, x2, y2]=point_dist(samp_points_2d(m,1), samp_points_2d(m,2), (-1/slopes(k)), len);
                x1=samp_points_2d(m,1)+len/(sqrt((-1/slopes(k))^2+1));
                x2=samp_points_2d(m,1)-len/(sqrt((-1/slopes(k))^2+1));
                y1=(-1/slopes(k))*(x1-samp_points_2d(m,1))+samp_points_2d(m,2);
                y2=(-1/slopes(k))*(x2-samp_points_2d(m,1))+samp_points_2d(m,2);
                
                search_end_points(len+1,:)=round([x1 y1 x2 y2]);
                
                if search_end_points(len+1,1)<IRx+1 && search_end_points(len+1,1)>0 && search_end_points(len+1,2)<IRy+1 && search_end_points(len+1,2)>0
                if edgeim(search_end_points(len+1,2), search_end_points(len+1,1))
                    correspondence_final(h,:)=[search_end_points(len+1,1) search_end_points(len+1,2) ...
                        samp_points_3d(m,:) slopes(k)];
                    test_final(h,:)=[search_end_points(len+1,1) search_end_points(len+1,2) ...
                        samp_points_2d(m,:) samp_points_3d(m,:)];
                    h=h+1;
                    break; % for keeping multiple hypothesis, for keeping single hypothesis uncomment this line
                end
                end
                if search_end_points(len+1,3)<IRx+1 && search_end_points(len+1,3)>0 && search_end_points(len+1,4)<IRy+1 &&  search_end_points(len+1,4)>0
                if edgeim(search_end_points(len+1,4), search_end_points(len+1,3))
                    correspondence_final(h,:)=[search_end_points(len+1,3) search_end_points(len+1,4) ...
                        samp_points_3d(m,:) slopes(k)];
                    test_final(h,:)=[search_end_points(len+1,3) search_end_points(len+1,4) ...
                        samp_points_2d(m,:) samp_points_3d(m,:)];
                    h=h+1;
                    break; % for keeping multiple hypothesis, for keeping single hypothesis uncomment this line
                end
                end
            end
            end
        end
        k=k+1;
    end
end

if isempty(correspondence_final)
   correspondence_final_unique=[]; 
   correspondence_final=[]; %delete this
   test_final=[];
    return 
end

correspondence_final(:,1) = ((correspondence_final(:,1)-IRx/2)/IPPM); % convert to image coordinate from pixel coordinate
correspondence_final(:,2) = ((IRy/2 -correspondence_final(:,2))/IPPM); % convert to image coordinate from pixel coordinate
correspondence_final_unique=unique([correspondence_final(:,1:6) test_final(:,1:4)], 'rows'); % delete this and change in the function
end