function [cloud] = passthrough_filter(cloud)%
cloud((cloud(:,1)>0.5),:)=[];
cloud((cloud(:,1)<-0.5),:)=[];
cloud((cloud(:,2)<-0.5),:)=[];

% Cloud = pointCloud(cloud);
% %Additional Filtering
% gridStep = 0.001;%0.01
% ptCloudA = pcdownsample(Cloud,'gridAverage',gridStep);
% ptCloudOut = pcdenoise(ptCloudA,'Threshold',1.0);
% cloud = ptCloudOut.Location(:,:,:);
% pcwrite(Cloud,'/home/nus/Desktop/Panel.ply','Encoding','ascii');

% %% Reconstructed Workpiece
% cloud((cloud(:,1)>0.5),:)=[];
% cloud((cloud(:,1)<-0.5),:)=[];
% cloud((cloud(:,2)<-1),:)=[];
% cloud((cloud(:,2)>0.5),:)=[];

end
