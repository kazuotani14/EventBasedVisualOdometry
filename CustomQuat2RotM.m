function [rotm] = CustomQuat2RotM(quat)

	rotm = zeros(3);
	q_x = quat(1);
	q_y = quat(2);
	q_z = quat(3);
	q_w = quat(4);
	rotm(1,1) = 1 - 2*q_y^2 - 2*q_z^2;
	rotm(1,2) = 2*q_x*q_y - 2*q_z*q_w;
	rotm(1,3) = 2*q_x*q_z + 2*q_y*q_w;
	rotm(2,1) = 2*q_x*q_y + 2*q_z*q_w;
	rotm(2,2) = 1 - 2*q_x^2 - 2*q_z^2;
	rotm(2,3) = 2*q_y*q_z - 2*q_x*q_w;
	rotm(3,1) = 2*q_x*q_z - 2*q_y*q_w;
	rotm(3,2) = 2*q_y*q_z + 2*q_x*q_w;
	rotm(3,3) = 1 - 2*q_x^2 - 2*q_y^2;
end