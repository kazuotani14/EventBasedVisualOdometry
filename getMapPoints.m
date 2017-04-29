function [map_points] = getMapPoints(points_in_planes, kf_pose_estimate)%  - origin is implied to be (0,0,0)?
    % Input:
    % -------
    % points_in_planes: clustered points as h x w x 1
    % kf_pose_estimate: 3D pose estimates as 3 x 1
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