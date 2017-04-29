% This function should give an event image based off of the start time of collecting
% events. As specified, they wait till the number of points is a percentage of the
% map, and not more than one event hits a pixel. We do the latter.

%NOTE THE ABOVE IS THE IDEAL CASE, WE ARE ACTUALLY BOUNDING WITH THE GROUNDTRUTH
%DATA INSTEAD FOR THE TIME BEING

function [event_image, curr_pose_estimate, keyframe_bool] = GetEventImage(kf_pose_estimate, last_pose_estimate, curr_pose_estimate, event_mat)
event_image = zeros(180,240); %this is the size of the camera

start_time = last_pose_estimate(1);
end_time = curr_pose_estimate(1);

event_idx = find(event_mat(:,1) >= start_time,1);

%build event image while no pixel gets 2 events
% while max(event_image) < 2;	
time = 0;
while time < end_time
	%the plus ones are to deal with the fact that normal people zero index
	event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) = ...
		event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) + 1;
	event_idx = event_idx + 1;
	time = event_mat(event_idx,1);
end

%clean up the 2 event pixel
% event_idx = event_idx - 1;
% event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) = ...
% 	event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) - 1;
% end_time = event_mat(event_idx+1,1);

% figure;
% imagesc(event_image)

%clean up to be binary
event_image = event_image > 0;

% figure;
% imshow(event_image);

% calculate the norm of the distance to determine if we should do a keyframe
dist_thres = 0.01;
distance_from_kf = norm(curr_pose_estimate(2:end)-kf_pose_estimate(2:end));
keyframe_bool = distance_from_kf > dist_thres;

end
