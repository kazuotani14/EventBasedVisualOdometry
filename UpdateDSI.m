function [KF_DSI] =  UpdateDSI(KF_DSI, event_image, T_kf, T_i, KF_Homographies, KF_depths, calib)
% Transformation_to_KF should transform an event_image in to th keyframe
% event_image is 180x240(original) or 231x309 (corrected)


%find first homography from KF image frame to the z0 plane
n_planes = size(KF_DSI,3);

Fx = calib.fx * 18.5 * 10e-6; Fy = calib.fy * 18.5 * 10e-6;
cx = calib.cx; cy = calib.cy;
fx = calib.fx; fy = calib.fy;
K = [fx, 0, cx;
    0, fy, cy;
    0, 0, 1];


H_i = K * [eye(3),zeros(3,1)] * T_i;
H_kf = K * [eye(3),zeros(3,1)] * T_kf;

H_i2kf = H_kf / H_i;

% p_i = [52,81,1]';
% p_kf = H_i2kf*p_i;


%%

H = cell(n_planes,1);

for i=1:n_planes
    d = KF_depths(i);
    H{i} = KF_Homographies{i}; %[d/-Fx, 0, 0; 0, d/-Fy, 0; 0, 0, 1]*H_i2kf; % This is what it should be
end

for i=1:n_planes
%    tform = projective2d(H{i}');
%    event_im_KF = imwarp(event_image, tform, 'OutputView',imref_obj);
   event_im_KF = warpH(event_image, H{i}, [size(KF_DSI,1),size(KF_DSI,2)]);
   spy(sparse(KF_DSI(:,:,i)));
   disp('update DSI');
%    pause(0.1)
   % increment relevant DSI voxels
   KF_DSI(:,:,i) = KF_DSI(:,:,i) + event_im_KF;
end


end