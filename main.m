%% plane detection and object segmentation
%   Author:  	    Leon de Lange 
%                   TuDelft, BMD Master Thesis
%                   Multi-view object retrieval
%   Last revision:  15 march 2016


clc
close all


%% constants
res = [640 480];            % [px] kinect resolution
state = eye(4);             % kinect motion matrix state
trace = [0 0 0]';           % kinect trace


%% kinect RGB/IR camera intrinsic and extrinsic parameters
% intrinsic matrix ir-depth camera
intr_ir = [     572     0       315;
                0       542     240;
                0       0       1       ];
        
% intrinsic matrix rgb camera
intr_rgb = [    511     0       322;
                0       493     249;
                0       0       1       ];

% constants: linear relation disparity to inverse depth coordinates
k = [-0.00307   3.33095]; 

% image indices (x,y) based on kinect resolution
ind = [repmat((1:res(1))',[res(2),1]) repelem((1:res(2))',res(1)) ones(res(1)*res(2),1)]; 

% inverse depth coordinates
ir_uv = (intr_ir\ind')';

% extrinsic camera parameters
R = [   0.9998      0.0013      -0.0175;
        -0.0015     0.9999      -0.0123;
        0.0175      0.0123      0.9998      ];

t = [   0.0200      -0.0007     -0.0109     ]';

extr = [R t; [0 0 0 1]];

% registration parameters
reg.res = res;              % [px] kinect resolution
reg.k = k;                  % linear relation disparty and inverse depth
reg.ir_uv = ir_uv;          % inverse depth coordinates
reg.intr_rgb = intr_rgb;    % intrinsic parameters rgb camera
reg.extr = extr;            % extrinsic parameters


%% create listener for kinect camera
fprintf('Making first call to initalize the kinect driver and listening thread\n');
kinect_mex();
pause(3)

fprintf('Making second call starts getting data\n');
kinect_mex();
pause(2)


%% create video player
videoPlayer  = vision.VideoPlayer('Position',[100 100 [res(1), res(2)]]);


%% forever loop
while (1==1)      
            
    % grab frame
    [unreg_d, rgb] = kinect_mex();
              
    % reshape datastream using kinect resolution
    rgb = permute(reshape(rgb,[3 res]),[3 2 1]);
    unreg_d = im2double(unreg_d, 'indexed');           
            
    % obtain registered depth point cloud
    pc = registration(unreg_d, reg);
        
    % calculate frames per second
    tic
    
    % find dominand plane
    [dist, pnorm] = detect_plane(pc);

    % obtain tha best plane
    distance = pc(:,:,1).*pnorm(1) + pc(:,:,2).*pnorm(2) + pc(:,:,3).*pnorm(3) - dist;
    
    % show objects in front of the plane
    mask_z = ~(distance > - 0.025);
         
    % show video
    rgb = im2double(rgb);   
    rgb(:,:,1) = rgb(:,:,1).*mask_z;
    rgb(:,:,2) = rgb(:,:,2).*mask_z;
    rgb(:,:,3) = rgb(:,:,3).*mask_z;
    step(videoPlayer, rgb);
               
    % time
    fps = 1/toc;
    
    clc
    display(fps)
    display(dist)   
    
end