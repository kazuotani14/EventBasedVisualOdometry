%This function should generate the inital projections from the keyframe onto 
% 50 planes, and also provide the Homographies from the kf to the planes
function [KF_scaling, KF_homographies, KF_dsi, KF_depths] = DiscretizeKeyframe(KF_image, min_depth, max_depth, N_planes, calib)
    % Assume kf_image is already adjusted for camera center and projected
    % onto image plane

	fx = calib.fx;         % focal length in pixels
	fy = calib.fy;          % focal length in pixels

    KF_depths = linspace(min_depth, max_depth, N_planes);
    KF_dsi = repmat(KF_image,1,1,N_planes);
    KF_homographies = cell(N_planes,1);
    KF_scaling = zeros(N_planes,2);

    h_image = size(KF_image,1);
    w_image = size(KF_image,2);
    
    cone_angle_x = atan((w_image/2)/fx);  % half of FOV
    cone_angle_y = atan((h_image/2)/fy);  % half of FOV
    
    % Assuming uniform WxH
    % output KF_homographies should be changed to KF_scales
    for i = 1:N_planes
        d = KF_depths(i);
        frustumHeight = 2 * d * tan(cone_angle_y);
        frustumWidth  = 2 * d * tan(cone_angle_x);
        
        % "pixels" to meters. d_pixels*scale = d_world
        scale_x = frustumHeight/h_image;   
        scale_y = frustumWidth/w_image;
        if i==0:
            scale_x0 = scale_x;
            scale_y0 = scale_y;
        end
        KF_scaling(i,:) = [scale_x, scale_y];
        
        KF_dsi(:,:, i) = KF_image;
        KF_homographies{i} = [scale_x/scale_x0, 0, 0;
                              0, scale_y/scale_y0, 0;
                              0, 0, 1];
    end
    
%     frame_corners = [1 1 1; 1 h 1; w 1 1; w h 1];
%     [y,x] = ind2sub(size(kf_image),find(kf_image));

end