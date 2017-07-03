%%% START LOAD DATASET

global event_mat; % why is this here?

% Make sure data folder is added to path
% dataset = 'shapes_rotation';
% dataset = 'shapes_translation';
% dataset = 'dynamic_6dof';
dataset = 'boxes_6dof';

load(strcat(dataset,'_events.mat'), 'event_mat');
load(strcat(dataset,'_groundtruth.mat'), 'groundtruth_mat');
load(strcat(dataset,'_calib.mat'), 'calib');
disp('Done loading data!');

% For starting from specific points in the dataset
% event_mat = event_mat(94799:end, :);
% groundtruth_mat = groundtruth_mat(186:end, :);

%%% START VARIABLE INIT
end_time = 0;
% % This is a fake kf pose to simply force the first event image to be a keyframe
first_kf_pose = -1000*ones(1,8);

kf_pose_estimate = first_kf_pose;
last_pose_estimate = first_kf_pose;

groundtruth_idx = 1;
curr_pose_estimate = groundtruth_mat(groundtruth_idx,:);

N_planes = 50;  %Depth of DSI
min_depth = 0.75; % [m]
max_depth = 1.75; % [m]

KF_scaling = [];
KF_dsi = {};
KF_depths = [];
KF_count = 0;
frame_limit = 3000; % Limit number of event images used

map = [];

orig_calib = calib;

%%% MAIN LOOP
while end_time < event_mat(end-1,1)
    [event_image, curr_pose_estimate, keyframe_bool] = GetEventImage(kf_pose_estimate, last_pose_estimate, curr_pose_estimate);
    [event_image, calib] = CorrectDistortion(event_image, orig_calib);
%     imshow(event_image); disp('event image'); pause

    if keyframe_bool
        KF_count = KF_count + 1;
        fprintf('Keyframe: %d \t Map Size: %d \n',KF_count, size(map,1));
        % % Add old DSI points to global map and reset DSI
        if groundtruth_idx ~= 1
            depth_map = GetClusters(KF_dsi);
            depth_map = MedianFilterDepthMap(depth_map, [7,7]);
            [map_points] = GetNewMapPoints(depth_map, kf_pose_estimate, KF_scaling, KF_depths);
            if ~isempty(map_points)
                map_points = RadiusFilterMap(map_points, 0.1, min(0.06*size(map_points,1),15));
                map = [map; map_points];
            end
%             PlotResults(KF_dsi, depth_map, map);
        end
        % % Initialize new keyframe
        kf_pose_estimate = curr_pose_estimate;
        [KF_dsi, KF_scaling, KF_depths] = DiscretizeKeyframe(event_image, min_depth, max_depth, N_planes, calib);
    else
        % % Update DSI
        [T_kf, T_i] = FindPoseToKfH(kf_pose_estimate, curr_pose_estimate);
        [KF_dsi] =  UpdateDSI(KF_dsi, event_image, T_kf, T_i, KF_depths, calib);
    end

    groundtruth_idx = groundtruth_idx + 1;
    if mod(groundtruth_idx,25)==0
        fprintf('Event Image #%d\n',groundtruth_idx);
    end

    if mod(groundtruth_idx, frame_limit)==0
        break;
    end

    last_pose_estimate = curr_pose_estimate;
    curr_pose_estimate = groundtruth_mat(groundtruth_idx,:);
end


PlotResults(KF_dsi, depth_map, map);
