%This function should generate the inital projections from the keyframe onto 
% 50 planes, and also provide the Homographies from the kf to the planes
function [H_planes, H_tfs] = DiscretizeKeyframe(kf_image, kf_pose)

% for i = 1:50 %this is completely arbitrary

	frame_corners = [1 1 1; 1 size(kf_image,1) 1; size(kf_image,2) 1 1; size(kf_image,2) size(kf_image, 1) 1];

	kf_image = correct_distortion(kf_image);

	[y,x] = ind2sub(size(kf_image),find(kf_image));

	H_planes = cell(50,1);

	for i = 1:50
		H_planes{i} = [x, y, i*ones(size(x))]*inv(K_mat);
	end

	H_tfs = cell(50,1);

% end

end