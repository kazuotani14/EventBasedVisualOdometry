%This function is to take the pose of the kf and the pose of
% the event image and find the transforms between the two

% ref: http://16720.courses.cs.cmu.edu/lec/transformations.pdf
function [kf_M, i_M] = FindPoseToKfH(kf_pose, i_pose)

kf_T = kf_pose(2:4)';
kf_quat = kf_pose(5:8);
kf_R = CustomQuat2RotM(kf_quat);
kf_M = [kf_R, kf_T; 0 0 0 1];

i_T = i_pose(2:4)';
i_quat = i_pose(5:8);
i_R = CustomQuat2RotM(i_quat);
i_M = [i_R, i_T; 0 0 0 1];

end