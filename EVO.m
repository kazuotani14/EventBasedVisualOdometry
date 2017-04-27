load shapes_translation_events.mat
load shapes_translation_groundtruth.mat

[kf_times] = GetKeyFrameTimes(groundtruth_mat);

% We need to do a sort of fake bootstrap to start the map?, so
% we can take all the images before the first keyframe
% We pretend that the first image we get is a keyframe.
% I'm actually not sure what to do here because our groundtruth 
% trajectory times are not aligned with event images.
start_time = 0;
end_time = groundtruth_mat(1,1);
[first_image, end_time] = GetEventImage(start_time, end_time, event_mat);

[H_planes, H_tfs] = DiscretizeKeyframe(first_image, groundtruth_mat(1,2:end));

groundtruth_idx = 2;
while end_time < 1.3
	% DiscretizeKeyframe()
	[new_image, end_time] = GetEventImage(end_time, groundtruth_mat(groundtruth_idx,1), event_mat);
	% figure;
	imshow(new_image);
	groundtruth_idx = groundtruth_idx + 1;
end