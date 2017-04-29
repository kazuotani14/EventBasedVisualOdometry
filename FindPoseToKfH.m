%This function is to take the pose of the kf and the pose of
% the event image and find the transforms between the 2

% ref: http://16720.courses.cs.cmu.edu/lec/transformations.pdf
function [H_kis] = FindPoseToKfH(kf_pose, i_pose, calib)
	fx = calib.fx; fy = calib.fy; 
    cx = calib.cx; cy = calib.cy;

	K = [fx, 0, 0;
    	0, fy, 0;
    	cx, cy, 1]; %Intrinsics MATLAB convention

    kf_T = kf_pose(2:4)';
    kf_quat = kf_pose(5:8);
    kf_R = custom_quat2rotm(kf_quat);
    % kf_R = quat2rotm(kf_quat);
%     kf_H = K'*[kf_R, kf_T];
    kf_H = K'*[kf_R(:,1:2), kf_T];
    
    i_T = i_pose(2:4)';
    i_quat = i_pose(5:8);
    i_R = CustomQuat2RotM(i_quat);
    % i_R = quat2rotm(i_quat);
%     i_H = K'*[i_R, i_T];
    i_H = K'*[i_R(:,1:2), i_T];
    
    H_kis = kf_H/i_H;
end