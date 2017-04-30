function [KF_DSI] =  UpdateDSI(KF_DSI, H_to_KF, event_image, KF_Homographies, KF_depths, calib)
% Transformation_to_KF should transform an event_image in to th keyframe
% event_image is 180x240(original) or 231x309 (corrected)


%find first homography from KF image frame to the z0 plane
n_planes = size(KF_DSI,3);

% r1 = H_to_KF(:,1);
% r2 = H_to_KF(:,2);
% r3 = H_to_KF(:,3);
% t = H_to_KF(:,4);
z0 = KF_depths(1);
fx = calib.fx * 18.5 * 10e-6; fy = calib.fy * 18.5 * 10e-6;

H0 = [z0/fx 0 0; 0 z0/fy 0; 0 0 1];

%apply homographies to event image to get pixels in all z0-z50 planes

    for i = 1:length(KF_Homographies)
        H = KF_Homographies{i}*H0*H_to_KF;
        % transform event image to keyframe pose
        tform = projective2d(H');
        
        event_im_KF = imwarp(event_image, tform);
        imshow(event_im_KF);
        disp('update DSI');
        pause

        % increment relevant DSI voxels
        KF_DSI(:,:,i) = KF_DSI(:,:,i) + event_im_KF;

    end

end