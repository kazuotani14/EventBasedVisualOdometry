%==== Extract data from dataset and save as MAT file ====%

% dataset = 'shapes_rotation';
% dataset = 'shapes_translation';
% dataset = 'dynamic_6dof';
dataset = 'boxes_6dof';

%% Extract Event Data

tic
event_mat = dlmread(strcat('../data/',dataset,'/events.txt'));
save(strcat(dataset,'_events.mat'), 'event_mat', '-v7.3');
toc

%% Extract Ground Truth Data

tic
groundtruth_mat = dlmread(strcat('../data/',dataset,'/groundtruth.txt'));
save(strcat(dataset,'_groundtruth.mat'), 'groundtruth_mat', '-v7.3');
toc

%% Extract Calibration Data

tic
param = dlmread(strcat('../data/',dataset,'/calib.txt'));
calib = struct();
calib.fx = param(1);
calib.fy = param(2);
calib.cx = param(3);
calib.cy = param(4);
calib.k1 = param(5);
calib.k2 = param(6);
calib.p1 = param(7);
calib.p2 = param(8);
calib.k3 = param(9);
save(strcat(dataset,'_calib.mat'), 'calib', '-v7.3');
toc

