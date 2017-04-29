%This function will take high density places in the DSI,
% and return these points in 3D for addition to the map

function [new_map_points] = GetNewMapPoints(points_in_planes, kf_pose_estimate, KF_scaling, KF_depths)%  - origin is implied to be (0,0,0)?
    % Input:
    % -------
    % points_in_planes: clustered points as h x w x 1
    % kf_pose_estimate: (timestamp px py pz qx qy qz qw)
    %
    % Output:
    % -------
    % map_points: set of points as 3D coordinates h*w x 3
    tform = affine3d([cat(2, eye(3), zeros(3,1));...
                      kf_pose_estimate', 1]);
	h = size(points_in_planes, 1);
    w = size(points_in_planes, 2);
    [X, Y] = meshgrid(1:w, 1:h);
    X = X(:);
    Y = Y(:);
	Z = points_in_planes(:);
    [U, V, W] = transformPointsInverse(tform, X, Y, Z);
    map_points = [U, V, W];
end