clear;
clc;

%% Read the text files containing the visible edges 
visible_vertices = dlmread('FILE\LOCATION\visible_vertices_all_frames.txt');
all_edges = dlmread('FILE\LOCATION\visible_edges_whole_mesh.txt');

%% Check edges from the whole mesh
for k=1:max(visible_vertices(:,6))
        aa= find( visible_vertices(:,6) ==k);
        visible_vertices_framewise{k}=visible_vertices(aa,1:4);
    k % Display progress
end

%% Copy edges with [ID, X1,Y1,Z1, X2, Y2, Z2]
for m=1:length(visible_vertices_framewise)
    visible_vertices= visible_vertices_framewise{m};
    m % Display progress
n = 1;
for i =1: length (all_edges)
    [a,b]= ismember(all_edges(i,2), visible_vertices(:,1));
    if a
        [c,d]= ismember(all_edges(i,3), visible_vertices(:,1));
        if c
            visible_edges_all_frames{m}(n,:) = [all_edges(i,1), visible_vertices(b,2:4), visible_vertices(d,2:4)] ;
            n= n + 1;
        end
    end
end
end

%% save the results to a .mat file
save( 'visible_edges.mat','visible_edges_all_frames')
