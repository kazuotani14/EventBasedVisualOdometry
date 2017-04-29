%This function is to take the pose of the kf and the pose of
% the event image and find the transforms between the 2

% ref: http://16720.courses.cs.cmu.edu/lec/transformations.pdf
function [H_kis] = FindPoseToKfH(kf_pose, i_pose, intrinsic_K)
    kf_T = kf_pose(2:4)';
    kf_quat = kf_pose(5:8);
    kf_R = my_quat2rotm(kf_quat);
    kf_H = intrinsic_K*[kf_R, kf_T];
%     kf_H = K*[kf_R(1:2,:), kf_T];
    
    i_T = i_pose(2:4);
    i_quat = i_pose(5:8);
    i_R = my_quat2rotm(i_quat);
    i_H = intrinsic_K*[i_R, i_T];
%     i_H = K*[i_R(1:2,:), i_T];
    
    H_kis = kf_H/i_H;
end

function M = my_quat2rotm(quat)
    qw = quat(4);
    qx = quat(1);
    qy = quat(2);
    qz = quat(3);
    
    % quat seems normalized from data, may not need this
    % *************
    n = 1.0/sqrt(qx^2+qy^2+qz^2+qw^2);
    qw = n*qw;
    qx = n*qx;
    qy = n*qy;
    qz = n*qz;
    % *************
    
    M = [1-2*qy^2-2*qz^2, 2*qx*qy-2*qz*qw, 2*qx*qz + 2*qy*qw;
    2*qx*qy+2*qz*qw, 1-2*qx^2-2*qz^2, 2*qy*qz - 2*qx*qw;
    2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx^2-2*qy^2];
end