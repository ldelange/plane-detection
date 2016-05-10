function [reg_d] = registration(unreg_d, reg)
%% ir-rgb depth registration kinect camera
%   Author:  	    Leon de Lange 
%                   TuDelft, BMD Master Thesis
%                   Multi-view object retrieval
%   Last revision:  15 march 2016


%% constants
res = reg.res;              % [px] kinect resolution
k = reg.k;                  % linear relation disparty and inverse depth
ir_uv = reg.ir_uv;          % inverse depth coordinates
intr_rgb = reg.intr_rgb;    % intrinsic parameters rgb camera
extr = reg.extr;            % extrinsic parameters


%% camera depth registration
% ir-camera inverse depth coordinates
ir_q = k(1) .* unreg_d + k(2);
ir_q = ir_q .* ((ir_q) > 0);     

% ir-camera euclidean coordinates x, y and z
ir_z = 1 ./ ir_q;
ir_x = ir_z .* ir_uv(:,1);
ir_y = ir_z .* ir_uv(:,2);

% grab pointcloud in ir-camera frame
pc_ir = [ir_x ir_y ir_z ones(res(1)*res(2),1)];

% transform pointcloud to rgb-camera frame via extrinsic 
pc_rgb = (extr * pc_ir')';

% obtain inverse depth points rgb-camera
rgb_u = pc_rgb(:,1)./pc_rgb(:,3);
rgb_v = pc_rgb(:,2)./pc_rgb(:,3);

% project pointcloud on rgb-camera sensor
rgb_uv = (intr_rgb * [rgb_u rgb_v ones(res(1)*res(2),1)]')';

% get boundary mask
mask = rgb_uv(:,1) > 1 & rgb_uv(:,1) < res(1) & rgb_uv(:,2) > 1 & rgb_uv(:,2) < res(2);    

% get rgb indeces of rgb_uv coordaintes
indices = sub2ind([res(2) res(1)],floor(rgb_uv(mask,2)),floor(rgb_uv(mask,1))); 

% registered depth matrix
reg_x = zeros(res(2),res(1));
reg_x(indices) = pc_rgb(mask,1);
reg_y = zeros(res(2),res(1));
reg_y(indices) = pc_rgb(mask,2);
reg_z = zeros(res(2),res(1));
reg_z(indices) = pc_rgb(mask,3);

% concatenate registered depth layers
reg_d = cat(3,reg_x,reg_y,reg_z);