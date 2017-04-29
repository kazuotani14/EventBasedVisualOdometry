load shapes_translation_events.mat
load shapes_translation_groundtruth.mat

%%%RANDOM NOTES OF BEN, PLEASE IGNORE THESE FEW LINES
% We need to do a sort of fake bootstrap to start the map?, so
% we can take all the images before the first keyframe
% We pretend that the first image we get is a keyframe.

%%%%%%%%%%%%%%%%%%%%%%% START VARIABLE INIT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end_time = 0;
% this is a fake kf pose ot simply force the first event image to be a keyframe
first_kf_pose = -1000*ones(1,8);

kf_pose_estimate = first_kf_pose;
last_pose_estimate = first_kf_pose;

groundtruth_idx = 1;
curr_pose_estimate = groundtruth_mat(groundtruth_idx,:);

N = 50; %Depth of DSI
W = 309; %Width of distortion corrected image
H = 231; %Height of distortion corrected image

kf_DSI = zeros(H,W,N);

map = [];
%%%%%%%%%%%%%%%%%%%%%%%%% END VARIABLE INIT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while end_time < event_mat(end,1);
	[event_image, curr_pose_estimate, keyframe_bool] = GetEventImage(kf_pose_estimate, last_pose_estimate, curr_pose_estimate, event_mat);
	event_image = correct_distortion(event_image);
	imshow(event_image);

	if keyframe_bool
		% add old DSI points to global map and reset DSI
		% [points_in_planes] = getDSIClusters(kf_DSI);
		% [map_points] = getMapPoints(points_in_planes, kf_pose_estimate)%  - origin is implied to be (0,0,0)?
		% map = [map; map_points];

		% Initialize new keyframe
		% kf_DSI = zeros(W,H,N);
		% kf_pose_estimate = curr_pose_estimate;
		% kf_homographies = discretizeKeyframe(kf_pose_estimate, min_depth, max_depth, N_planes);
	else
		% update DSI
		% [Transformation_to_KF] = getTransformationtoKF(curr_pose_estimate, KF_pose_estimate, K);
		% [kf_DSI] = UpdateDSI(kf_DSI, Transformation_to_KF, event_image);
	end

	groundtruth_idx = groundtruth_idx + 1;
	last_pose_estimate = curr_pose_estimate;
	curr_pose_estimate = groundtruth_mat(groundtruth_idx,:);
end