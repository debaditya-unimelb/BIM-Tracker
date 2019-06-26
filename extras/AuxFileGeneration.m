clear;
load('visible_edges.mat');
imageNames = dir(fullfile('C:\Users\acharyad\Desktop\Research\Video\cases\640x480-56D-RealData_new\sequence','*.png'));
imageNames = {imageNames.name}';
for n=1:length(imageNames)
    im(:,:,n) = rgb2gray(imread(fullfile('C:\Users\acharyad\Desktop\Research\Video\cases\640x480-56D-RealData_new\sequence',imageNames{n})));
    im_bin(:,:,n)=edge(im(:,:,n), 'Canny', [0 0.15], 1.4);
    edgeim(:,:,n)=im_bin(:,:,n);
end
save( 'aux_data_real.mat','edgeim', 'visible_edges_all_frames')