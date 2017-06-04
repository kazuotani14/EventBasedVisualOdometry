%This function should generate the inital projections from the keyframe onto 
% 50 planes, and also provide the Homographies from the kf to the planes
function [KF_dsi, KF_scaling, KF_depths] = DiscretizeKeyframe(KF_image, min_depth, max_depth, N_planes, calib)
    % Assume kf_image is already adjusted for camera center and projected
    % onto image plane

	fx = calib.fx;         % focal length in pixels
	fy = calib.fy;          % focal length in pixels

    % For uniform depth
    KF_depths = linspace(min_depth, max_depth, N_planes);
   
    % For uniform inverse depth
    s = (1/min_depth - 1/max_depth)/N_planes;
    inv_d = linspace(1/min_depth, 1/max_depth, 50);
%     KF_depths = 1./inv_d;
    
    KF_dsi = zeros(size(KF_image,1), size(KF_image,2) ,N_planes);
    KF_scaling = zeros(N_planes,2);

    h_image = size(KF_image,1);
    w_image = size(KF_image,2);
    
    cone_angle_x = atan((w_image/2)/fx);  % half of FOV
    cone_angle_y = atan((h_image/2)/fy);  % half of FOV
    
    % Assuming uniform WxH
    for i = 1:N_planes
        d = KF_depths(i);
        frustumHeight = 2 * d * tan(cone_angle_y);
        frustumWidth  = 2 * d * tan(cone_angle_x);
        
        % "pixels" to meters. d_pixels*scale = d_world
        scale_x = frustumHeight/h_image;   
        scale_y = frustumWidth/w_image;
        KF_scaling(i,:) = [scale_x, scale_y];
        
    end
    
end