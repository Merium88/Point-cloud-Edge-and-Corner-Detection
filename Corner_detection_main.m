%% Read Point cloud
ptCloud = pcread('/media/mariam/Mariam_Backup/IROS 2018 Dataset/PanelLine Dataset/Old dataset/panel_line3.pcd');%'/media/nus/Umer Rasheed 83110809/PanelLine Dataset/panel_line3.pcd'
X= ptCloud.Location(:,:,:);
%% For Panel line work piece 
C = reshape(X,[size(X,1)*size(X,2),size(X,3)]);
C(isnan(C),:) = 0;
idx = find(C(:,1)==0);
C(idx,:)=[];
filtered_pcd = passthrough_filter(C);
Cloud = pointCloud(filtered_pcd);
gridStep = 0.01;
ptCloudA = pcdownsample(Cloud,'gridAverage',gridStep);
C = ptCloudA.Location(:,:,:);
pcshow(ptCloudA);axis on
%% Edge detection
[edge_points,edge_id,curve,surface_normal,cloud] = edgedetection(C);%
scatter3(edge_points(:,1),edge_points(:,2),edge_points(:,3),'.');
%% Corner detection
corner_detection(cloud,edge_id,edge_points,curve,surface_normal,i);%[edge_points,edge_id,curve,surface_normal]%,labels
