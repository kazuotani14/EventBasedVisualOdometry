% This function takes in the groundtruth data and figures 
% the time stamps of when keyframes need to be created
function [kf_times] = GetKeyFrameTimes(groundtruth_mat)
% the paper suggests that the keyframes should be created at
% particular distance intervals depending on the average depth
% of the map. We are going just to do this slightly differently,
% we will have a sort of hard coded threshold.

dist_thres = 0.01; % not sure what this should be

num_poses = size(groundtruth_mat, 1);
kf_times = [];

i = 1;
while i <= num_poses
	distance_accum = 0;
	while (distance_accum < dist_thres) & (i <= num_poses)
		if i ~= 1
			distance_accum = distance_accum + norm(groundtruth_mat(i-1,:)-...
				groundtruth_mat(i,:));
        end
		i = i + 1;
    end
    kf_times = [kf_times groundtruth_mat(i-1,1)];
end

end