%This function will take high density places in the DSI,
% and return these points in 3D for addition to the map

function [new_map_points] = GetNewMapPoints(depth_map, kf_pose_estimate, KF_scaling, KF_depths)%  - origin is implied to be (0,0,0)?
    % Input:
    % -------
    % depth_map: clustered points as h x w x 1, where the values are the depth plane
    % kf_pose_estimate: (timestamp px py pz qx qy qz qw)
    %
    % Output:
    % -------
    % map_points: set of points as 3D coordinates number of valid points x 3

    kf_T = kf_pose(2:4)';
    kf_quat = kf_pose(5:8);
    kf_R = custom_quat2rotm(kf_quat);

	tform = [kf_R, kf_T;
			zeros(1,3), 1];

	valid_idx = find(depth_map);

	[valid_y, valid_x] = ind2sub(size(depth_map), valid_idx);

	points_in_camera_frame = [KF_scaling(depth_map(valid_y,valid_x),1)*valid_x, KF_scaling(depth_map(valid_y,valid_x),2)*valid_y, KF_scaling(depth_map(valid_y,valid_x))];

	new_map_points = points_in_camera_frame*tform';
end