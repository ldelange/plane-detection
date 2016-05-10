function [distance, dnorm] = detect_plane(pointcloud)
%% dominant plane detection
%   Author:  	    Leon de Lange 
%                   TuDelft, BMD Master Thesis
%                   Multi-view object retrieval
%   Last revision:  18 march 2016


%% constants
filter = ones(3,3)/9;       % smoothing filter surface normals
grid = 3;                   % voxel grid surface normals
csize = 500;                % cluster size (normals)
merge = 0.75;               % maximum vector distance for cluster merging


%% plane detection
% resize pointcloud to 1/3 of its size for faster plane estimation
pointcloud = pointcloud(1:3:size(pointcloud,1),1:3:size(pointcloud,2),:);

% new pointcloud size
pc_y = size(pointcloud,1);
pc_x = size(pointcloud,2);

% create mask for assigned depth values
mask = (pointcloud(:,:,3) ~= 0);

% compute surface normals
[nx, ny, nz] = surfnorm(pointcloud(:,:,1),pointcloud(:,:,2),pointcloud(:,:,3));

% smooth surface normals
nx = imfilter(nx,filter);
ny = imfilter(ny,filter);
nz = imfilter(nz,filter);

% define edges of surface normals -1 to 1
edges = -1.01:2.02/grid:1.01;

% place surface normals in histogram
[~,vox_x] = histc(nx,edges);
[~,vox_y] = histc(ny,edges);
[~,vox_z] = histc(nz,edges);

% create 3D voxel grid output
voxels = (vox_x + (vox_y-1).*grid + (vox_z-1).*grid^2);

% obtain direction dominant plane normal
direction = mode(voxels(mask));

% create 3D histogram of all surface normals
edges = (0:1:grid^3)+0.5;
h = histc(voxels(mask),edges);

% obtain voxels with bincount larger than csize
clusters = find(h > csize);

% when no clusters are selected
if isempty(clusters)
    
    % add maximum direction
    clusters = direction;
    
end

% for each cluster
cavg = zeros(length(clusters),3);
for i = 1:length(clusters)

    % calculate cluster mask
    cmask = (voxels == clusters(i));

    % obtain cluster average normal
    cavg(i,:) = [mean(nx(cmask)) mean(ny(cmask)) mean(nz(cmask))];

end

% average vector dominant direction repeated for each cluster
davg = repmat(cavg(clusters == direction,:),[length(clusters),1]);

% distances to average cluster vector
dclust = sqrt(sum((davg-cavg).^2,2));

% merge neighbour clusters
directions = clusters(dclust < merge);
    
% create output mask of all merged clusters
out = zeros(pc_y,pc_x);
for i = 1:length(directions)

    % mask dominant direction
    out = out + (voxels == directions(i));
    
end

% obtain cluster size of merged clusters
bincount = h(clusters(dclust<merge));

% calculate dominant plane vector based on cluster size
dnorm = (cavg(dclust<merge,:)'*bincount)/sum(bincount,1);

% normalize plane normal
dnorm = dnorm/norm(dnorm);

% point to plane distance
planedist = pointcloud(:,:,1).*out.*dnorm(1) + pointcloud(:,:,2).*out.*dnorm(2) + pointcloud(:,:,3).*out.*dnorm(3);

% create distance histogram
edges = 0:0.025:3;
h = histc(planedist(logical(out)),edges);

% find index most common edge distance
distance = edges(find(h == max(h)));
distance = distance(1);

end