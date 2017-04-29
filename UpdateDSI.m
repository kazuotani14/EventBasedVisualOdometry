function [KF_DSI] =  UpdateDSI(KF_DSI, Transformation_to_KF, event_image)
% Transformation_to_KF should transform an event_image in to th keyframe
% event_image is 180x240(original) or 231x309 (corrected)

% transform event image to keyframe pose
tform = affine2d(Transformation_to_KF);
event_im_KF = imwarp(event_image, tform);

% backproject event image into DSI


% increment relevant DSI voxels 



end