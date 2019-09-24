function [thin_edge,edge,curve,surface_normal,cloud] = edgedetection(cloud)%
res = 4.5;%4
NearestN = 400;%100;500(shapenet)
% Tree = createns(cloud,'NSMethod','kdtree','Distance','euclidean');

%% Compute Edge
tic
i=1:1:length(cloud);
[idx,D] = knnsearch(cloud,cloud(:,:),'k',NearestN);
NN = D(:,2);
idx = idx';
X = cloud(:,1);Y = cloud(:,2);Z = cloud(:,3);
x = X(idx);y = Y(idx);z = Z(idx);
centroid = [mean(x,1);mean(y,1);mean(z,1)]';
dist = sqrt(sum((cloud(:,:)-centroid).^2,2));
edge= nonzeros((dist> NN*res).*(i)');
disp(toc)

%% Edge Thinning
% n=1;flag = 0;l=1;
edge_cloud = cloud(edge,:);%filt_edge;
thin_edge = [];
i=1:1:length(edge_cloud);
point = edge_cloud(:,:);
[idx,D] = knnsearch(edge_cloud,point,'k',150);%50,200
idx = idx';
X = edge_cloud(:,1);Y = edge_cloud(:,2);Z = edge_cloud(:,3);
x = X(idx);y = Y(idx);z = Z(idx);
thin_edge = [mean(x,1);mean(y,1);mean(z,1)]';
dist = pdist2(thin_edge,thin_edge);
[row,col] = find(dist==0);
id = find(row~=col);
thin_edge(row(id),:)=[];edge(row(id),:)=[];


%% Additional Filtering
Cloud = pointCloud(thin_edge);
gridStep = 0.01;
ptCloudA = pcdownsample(Cloud,'gridAverage',gridStep);
% ptCloudB = pcdenoise(ptCloudA,'NumNeighbors',5,'Threshold',1);
edge_cloud = ptCloudA.Location(:,:,:);
point = edge_cloud(:,:);
% 
FullCloud = pointCloud(cloud);
gridStep = 0.005;%0.01
ptCloudA = pcdownsample(FullCloud,'gridAverage',gridStep);
% ptCloudB = pcdenoise(ptCloudA);
cloud = ptCloudA.Location(:,:,:);

[idx,D] = knnsearch(cloud,point,'k',1);%50
% a = find(D(:)>0.005);
% idx(a) = [];
new_edge=idx;
edge_points = cloud(idx,:);
thin_edge = edge_points;edge = new_edge;

%% Compute Curvature & surface normal
n=1; 
point = cloud(edge,:);
[idx,D] = knnsearch(thin_edge,point,'k',10);
idx = idx';
X = thin_edge(:,1);Y = thin_edge(:,2);Z = thin_edge(:,3);
NN_x = X(idx(:,:));NN_y = Y(idx(:,:));NN_z = Z(idx(:,:));
for i=1:1:length(thin_edge)
NN = [NN_x(:,i),NN_y(:,i),NN_z(:,i)];
[z,mu,sigma]= zscore(NN);
mn = bsxfun(@minus,NN,mu);

% R = max(D(n,:));
% 
% for i = 1:1:3
%  for j = 1:1:3
% 	covarianceMatrix(i,j) = 0.0;
% 	 for k=1:1:length(NN(:,1))
% 		covarianceMatrix(i,j) = covarianceMatrix(i,j) + (R-D(n,k))*(mu(i) - NN(k,i))*(mu(j) - NN(k,j));
% 		covarianceMatrix(i,j) = covarianceMatrix(i,j)/(length(NN(:,1)) - 1);
%      end
%      covarianceMatrix(i,j) = covarianceMatrix(i,j)/sum(abs(R-D(n,:)));
%  end
% end
% 
% [U,S,V] = svd(covarianceMatrix) ;
% [~,id] = max(diag(S));
% eigen_val = diag(S);

[coeff,score,latent] = pca(mn);
curve(n,:)= coeff(:,1);%U(:,id);%
n=n+1;
end

% % 
% hold on
% quiver3(thin_edge(:,1),thin_edge(:,2),thin_edge(:,3),curve(:,1),curve(:,2),curve(:,3));

% % hold on
% % quiver3(thin_edge(:,1),thin_edge(:,2),thin_edge(:,3),curve(:,1),curve(:,2),curve(:,3));
% for k=1:1:length(curve(:,1))
%  u = curve(k,:)';v = curve1(k,:)';
%  Theta = (atan2(norm(cross(u,v)),dot(u,v)))*180/3.142;
%  i=0:5:180;
%  Curve_hist(k,:) = hist(Theta,i);%
%  if(sum(Curve_hist(k,3:35))>1)
%      Corner(k) = 1; 
%      else
%      Corner(k) = 0;
%  end
% 
% end
% 
% Corner_id = find(Corner==1);
% plot3(thin_edge(:,1),thin_edge(:,2),thin_edge(:,3),'.','MarkerSize',10);axis off
% hold on
% plot3(thin_edge(Corner_id,1),thin_edge(Corner_id,2),thin_edge(Corner_id,3),'.','MarkerSize',10);axis off
% hold on
% quiver3(thin_edge(:,1),thin_edge(:,2),thin_edge(:,3),curve(:,1),curve(:,2),curve(:,3));
% hold on
% quiver3(thin_edge(:,1),thin_edge(:,2),thin_edge(:,3),curve1(:,1),curve1(:,2),curve1(:,3));
% 

%% Surface normal for all points
n=1;
point = cloud(:,:);
[idx,D] = knnsearch(cloud,point,'k',10);
idx = idx';
X = cloud(:,1);Y = cloud(:,2);Z = cloud(:,3);
NN_x = X(idx(:,:));NN_y = Y(idx(:,:));NN_z = Z(idx(:,:));
for n=1:1:length(point(:,1))
NN = [NN_x(:,n),NN_y(:,n),NN_z(:,n)];
[z,mu,sigma]= zscore(NN);
mn = bsxfun(@minus,NN,mu);%subtract mean

%% Using weighted covariance matrix
% R = max(D(n,:));
% 
% for i = 1:1:3
%  for j = 1:1:3
% 	covarianceMatrix(i,j) = 0.0;
% 	 for k=1:1:length(NN(:,1))
% 		covarianceMatrix(i,j) = covarianceMatrix(i,j) + (R-D(n,k))*(mu(i) - NN(k,i))*(mu(j) - NN(k,j));
% 		covarianceMatrix(i,j) = covarianceMatrix(i,j)/(length(NN(:,1)) - 1);
%      end
%      covarianceMatrix(i,j) = covarianceMatrix(i,j)/sum(abs(R-D(n,:)));
%  end
% end
% 
% [U,S,V] = svd(covarianceMatrix) ;
% [~,id] = min(diag(S));
% eigen_val = diag(S);

[coeff,score,latent] = pca(mn);
surface_normal(n,:)= coeff(:,3);%U(:,id);%
n=n+1;
end
% plot3(cloud(edge,1),cloud(edge,2),cloud(edge,3),'.','MarkerSize',10);axis off

end
