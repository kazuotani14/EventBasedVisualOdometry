% This function should give an event image based off of the start time of collecting
% events. As specified, they wait till the number of points is a percentage of the
% map, and not more than one event hits a pixel. We do the latter.

%NOTE THE ABOVE IS THE IDEAL CASE, WE ARE ACTUALLY BOUNDING WITH THE GROUNDTRUTH
%DATA INSTEAD FOR THE TIME BEING

function [event_image, curr_pose_estimate, keyframe_bool] = GetEventImage(kf_pose_estimate, last_pose_estimate, curr_pose_estimate)

global event_mat;

event_image = zeros(180,240); %this is the size of the camera

start_time = last_pose_estimate(1);
end_time = curr_pose_estimate(1);

start_idx = find(event_mat(:,1) >= start_time, 1);
end_idx = find(event_mat(:,1) >= end_time, 1);

event_idxs = start_idx:end_idx;
r = event_mat(event_idxs,3)+1;
c = event_mat(event_idxs,2)+1;
pix_idxs = sub2ind([180,240], r, c);

event_image(pix_idxs) = 1;

dist_thres = 0.05;
distance_from_kf = norm(curr_pose_estimate(2:end)-kf_pose_estimate(2:end));
keyframe_bool = distance_from_kf > dist_thres;

end
