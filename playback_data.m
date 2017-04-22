% TODO
% Find more efficient way to play data, while allowing us to overlay
% markers on top
% Integrate DVS events - may need to pass events thru calibration matrix

function [] = playback_data()

directory = 'data/shapes_rotation/images/';

start_frame = 1;
end_frame = 1356;

% outputVideo = VideoWriter(fullfile('.','shapes_rotation_out.avi'));
% outputVideo.FrameRate = 22.69374787246113695676841030296153409735617837285827754453;
% open(outputVideo)

for framenum = start_frame:end_frame
    filepath = frameNum2filePath(directory, framenum);
    frame = imread(filepath);
    % writeVideo(outputVideo,frame)
    imshow(frame);
    disp(framenum)
end

end

function filepath = frameNum2filePath(directory, framenum)
    n_digits = floor(log10(framenum)) + 1;
    pad_zeros = repmat('0',1, 8-n_digits);
    filepath = strcat(directory, '/frame_', pad_zeros, num2str(framenum) , '.png');
end