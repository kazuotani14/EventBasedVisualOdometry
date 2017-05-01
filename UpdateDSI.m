function [KF_DSI] =  UpdateDSI(KF_DSI, event_image, T_kf, T_i, KF_Homographies, KF_depths, calib)
 
% Transformation_to_KF should transform an event_image in to th keyframe
% event_image is 180x240(original) or 231x309 (corrected)
 
%find first homography from KF image frame to the z0 plane
n_planes = size(KF_DSI,3);

cx = calib.cx; cy = calib.cy;
fx = calib.fx; fy = calib.fy;
K = [fx, 0, cx;
    0, fy, cy;
    0, 0, 1];
 
 
H_i = K * [eye(3),zeros(3,1)] * T_i;
H_kf = K * [eye(3),zeros(3,1)] * T_kf;
 
H_i2kf = H_kf / H_i;
 
kf_R = T_kf(1:3,1:3);
i_R  = T_i(1:3, 1:3);
kf_T = T_kf(1:3, 4);
i_T  = T_i(1:3, 4);
R_i2kf = kf_R*i_R';
% R_i2kf = kf_R'*i_R;
 
T_i2kf = -kf_T + i_T;
T_kis = [R_i2kf, T_i2kf; 0 0 0 1];
 
R_kf2i = R_i2kf';
T_kf2i = -T_i2kf;
 
r1 = R_kf2i(:,1);
r2 = R_kf2i(:,2);
r3 = R_kf2i(:,3);
t = T_kf2i;
 
Cx = -dot(r1,t);
Cy = -dot(r2,t);
Cz = -dot(r3,t);
 
H_i2z0 = [R_kf2i(:,1), R_kf2i(:,2), KF_depths(1)*R_kf2i(:,3)+T_kf2i];
H_kf2z0 = inv(H_i2z0)*(H_i2kf);
H_kf2zo(3,3) = 1;
 
% p_i = [52,81,1]';
% p_kf = H_i2kf*p_i;
%%
 
H = cell(n_planes,1);
 
H_kf2z0 = eye(3); % This is what we need to determine!!
 
for i=1:n_planes
    zi = KF_depths(i);
%     H{i} = KF_Homographies{i}*H_kf2z1*H_i2kf;
    % Trying dilation
    delta = (zi-Cz) / (KF_depths(1)-Cz);
    dilation = [delta, 0 , (1-delta)*Cx;
                0, delta, (1-delta)*Cy;
                0, 0, 1];
    H{i} = dilation * H_i2z0;  % H_kf2z0 * H_i2kf; 
end
 
for i=1:n_planes
%    tform = projective2d(H{i}');
%    event_im_KF = imwarp(event_image, tform, 'OutputView',imref_obj);
   event_im_KF = warpH_rect(event_image, H{i}, [size(KF_DSI,1),size(KF_DSI,2)], KF_Homographies{i});
   % increment relevant DSI voxels
   KF_DSI(:,:,i) = KF_DSI(:,:,i) + event_im_KF;
   spy(sparse(KF_DSI(:,:,i)));
   disp('update DSI')
%    disp(sum(sum(KF_DSI(:,:,i))));
%    pause(0.1)
end
 
end