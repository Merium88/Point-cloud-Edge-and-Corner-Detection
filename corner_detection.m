function corner_detection(cloud,edge_id,edge,curve,normal,label,File_num)
count = 1;num=1;
%% find NN of edge point among cloud points in a patch
[id_e,D_e] = knnsearch(edge,edge(:,:),'k',20);%values used k=10,5,3
% [id_n,D_n] = knnsearch(cloud,edge(:,:),'k',50);%values used k=10,5,3


% figure
% plot3(edge(:,1),edge(:,2),edge(:,3),'.','MarkerSize',10);axis off
% hold on
% quiver3(edge(:,1),edge(:,2),edge(:,3),curve(:,1),curve(:,2),curve(:,3));
% Cloud = pointCloud(cloud);
% gridStep = 0.05;%0.01
% ptCloudA = pcdownsample(Cloud,'gridAverage',gridStep);
% ptCloudA = ptCloudA.Location(:,:,:);
% figure
% plot3(ptCloudA(:,1),ptCloudA(:,2),ptCloudA(:,3),'.','MarkerSize',5.0);axis off
% hold on
dist = 0.005;%0.005
for L=1:1:10
tic
for i=1:1:length(id_e(:,1))
% for l= 5:5:15
% patch = cloud(id_n(i,:)',:);
% patch_id = id_e(i,:)';
% [id,~,edge_p_id] = intersect(patch_id,edge_id);
edge_p = edge(id_e(i,:)',:);
% dist_e = D_e(i,:)'/norm(D_e(i,:)');
% dist_n = D_n(i,:)'/norm(D_n(i,:)');
curve_p = curve(id_e(i,:)',:);
% surface_p = normal(id_n(i,:)',:);

% Check if corner lies on 2/3 edges
X = edge_p(:,1);Y = edge_p(:,2);Z = edge_p(:,3);
diff_x = max(X)-min(X);
diff_y = max(Y)-min(Y);
diff_z = max(Z)-min(Z);
N = 0;
if(abs(diff_x)>dist && abs(diff_z)>dist && abs(diff_y)>dist)%
N = 3;
elseif((abs(diff_x)>dist && abs(diff_z)>dist)|| (abs(diff_x)>dist&& abs(diff_y)>dist)||(abs(diff_z)>dist&& abs(diff_y)>dist))
   N = 2; 
end
if(N~=0)
    
idx = kmeans(curve_p,N);
if(N==3)
u = mean(curve_p(idx==1,:),1);
v = mean(curve_p(idx==2,:),1);
w = mean(curve_p(idx==3,:),1);

size1 = length(find(idx==1));
size2 = length(find(idx==2));
size3 = length(find(idx==3));

Theta = (atan2(norm(cross(u,v)),dot(u,v)))*180/3.142;
Theta1 = (atan2(norm(cross(u,w)),dot(u,w)))*180/3.142;
Theta2 = (atan2(norm(cross(v,w)),dot(v,w)))*180/3.142;

    if((abs(size1-size2)<L || abs(size1-size3)<L || abs(size2-size3)<L))
        if((Theta>50 && Theta<150)&& (Theta1>50 && Theta1<150) && (Theta2>50 && Theta2<150))%
            pred(i) = 1;
        % plot3(edge_p(find(idx==1),1),edge_p(find(idx==1),2),edge_p(find(idx==1),3),'.','MarkerSize',15);
        % hold on
        % plot3(edge_p(find(idx==2),1),edge_p(find(idx==2),2),edge_p(find(idx==2),3),'.','MarkerSize',15);axis off
        % hold on
        % % plot3(edge_p(find(idx==3),1),edge_p(find(idx==3),2),edge_p(find(idx==3),3),'.','MarkerSize',15);axis off
        % % hold off
        % plot3(edge(find(pred==1),1),edge(find(pred==1),2),edge(find(pred==1),3),'.','MarkerSize',15);axis off
        % hold on
        else
            pred(i) = 0;
        end
    else
        pred(i) = 0;
    end
elseif(N==2)
u = mean(curve_p(idx==1,:),1);
v = mean(curve_p(idx==2,:),1);


size1 = length(find(idx==1));
size2 = length(find(idx==2));

Theta = (atan2(norm(cross(u,v)),dot(u,v)))*180/3.142;
    if(abs(size1-size2)<L)
        if((Theta>50 && Theta<150) )%
            pred(i) = 1;
        % plot3(edge_p(find(idx==1),1),edge_p(find(idx==1),2),edge_p(find(idx==1),3),'.','MarkerSize',15);
        % hold on
        % plot3(edge_p(find(idx==2),1),edge_p(find(idx==2),2),edge_p(find(idx==2),3),'.','MarkerSize',15);axis off
        % hold on
        % % plot3(edge_p(find(idx==3),1),edge_p(find(idx==3),2),edge_p(find(idx==3),3),'.','MarkerSize',15);axis off
        % % hold off
        % plot3(edge(find(pred==1),1),edge(find(pred==1),2),edge(find(pred==1),3),'.','MarkerSize',15);axis off
        % hold on
        else
            pred(i) = 0;
        end
    else
        pred(i) = 0;
    end
end
else
    pred(i) = 0;
end 

% if(pred(i)==0)
% plot3(edge_p(find(idx==1),1),edge_p(find(idx==1),2),edge_p(find(idx==1),3),'.','MarkerSize',15);
% hold on
% plot3(edge_p(find(idx==2),1),edge_p(find(idx==2),2),edge_p(find(idx==2),3),'.','MarkerSize',15);axis off
% hold on
% plot3(edge_p(find(idx==3),1),edge_p(find(idx==3),2),edge_p(find(idx==3),3),'.','MarkerSize',15);axis off
% end

% quiver3(edge_p(:,1),edge_p(:,2),edge_p(:,3),curve_p(:,1),curve_p(:,2),curve_p(:,3));
% end
Template(i,:) = [pred(i)];%label(i),
end
disp(toc)

if (num==1)
    list = [];
end

E_ind = find(Template(:,end)==1);%
E = edge(E_ind,:);%predicted crners

figure
Cloud = pointCloud(cloud);
gridStep = 0.020;%0.01
ptCloudA = pcdownsample(Cloud,'gridAverage',gridStep);
ptCloudA = ptCloudA.Location(:,:,:);
plot3(ptCloudA(:,1),ptCloudA(:,2),ptCloudA(:,3),'.','MarkerSize',5.0);axis off
hold on
plot3(edge(:,1),edge(:,2),edge(:,3),'.','MarkerSize',5,'MarkerFaceColor','green');axis off
hold on
plot3(E(:,1),E(:,2),E(:,3),'.','MarkerSize',25,'color','red');axis off

end
end