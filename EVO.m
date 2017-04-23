load shapes_translation_events.mat
load shapes_translation_groundtruth.mat

[kf_times] = GetKeyFrameTimes(groundtruth_mat);

% We need to do a sort of fake bootstrap to start the map?, so
% we can take all the images before the first keyframe
% We pretend that the first image we get is a keyframe.
% I'm actually not sure what to do here because our groundtruth 
% trajectory times are not aligned with event images.
start_time = 0;
[first_image, end_time] = GetEventImage(start_time, event_mat);
while end_time < kf_times(1)
	% DiscretizeKeyframe()
	[new_image, end_time] = GetEventImage(end_time, event_mat);
	% figure;
	% imshow(new_image);
end
