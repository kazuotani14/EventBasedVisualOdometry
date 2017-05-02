%correct camera distortion on event image
function [corrected_image, new_calib] = CorrectDistortion(event_image, calib)

	% [y,x] = ind2sub(size(event_image),find(event_image));

	% corrected_image = zeros(size(event_image));

	% [new_x, new_y] = arrayfun(@(u, v) correct_point(u, v), x, y);

	% corrected_image(new_y, new_x) = 1;

% 	fx = 199.092366542;         % focal length in pixels
% 	fy = 198.82882047;          % focal length in pixels
% 	cx = 132.192071378+1;       % optical center, in pixels
% 	cy = 110.712660011+1;       % optical center, in pixels
% 	k1 = -0.368436311798;       % radial distortion coefficient
% 	k2 = 0.150947243557;        % radial distortion coefficient
% 	p1 = -0.000296130534385;    % tangential distortion coefficient
% 	p2 = -0.000759431726241;    % tangential distortion coefficient
% 	k3 = 0.0;                   % radial distortion coefficient

    fx = calib.fx; fy = calib.fy; 
    cx = calib.cx; cy = calib.cy;
    k1 = calib.k1; k2 = calib.k2; k3 = calib.k3;
    p1 = calib.p1; p2 = calib.p2;

	K_mat = [fx, 0, 0;
	    	0, fy, 0;
	    	cx, cy, 1]; %Intrinsics MATLAB convention

	rad_dist_vec = [k1, k2, k3]; %radial distortion vector

	tang_dist_vec = [p1, p2];    %tangential distortion vector

	cameraParams = cameraParameters('IntrinsicMatrix', K_mat,...
                                    'RadialDistortion', rad_dist_vec,...
                                    'TangentialDistortion', tang_dist_vec);

	[corrected_image ,newOrigin] = undistortImage(event_image, cameraParams, 'nearest', 'OutputView', 'full');

	new_calib = calib
	new_calib.cx = new_calib.cx - newOrigin(1);
	new_calib.cy = new_calib.cy - newOrigin(2);

	% figure;
	% imshow(event_image);
	% figure;
	% imshow(corrected_image);
end

%% correct_point: function description
% function [x_u, y_u] = correct_point(x_d, y_d)
% 	x_c = 132.192071378+1;
% 	y_c = 110.712660011+1;
% 	k1 = -0.368436311798;
% 	k2 = 0.150947243557;
% 	p1 = -0.000296130534385;
% 	p2 = -0.000759431726241;
% 	k3 = 0.0;
% 	syms x;
% 	syms y;
% 	% https://en.wikipedia.org/wiki/Distortion_(optics)
% 	x_eqn = x_d == x*(1+k1*sqrt((x-x_c)^2+(y-y_c)^2)^2+k2*sqrt((x-x_c)^2+(y-y_c)^2)^4)+p2*(sqrt((x-x_c)^2+(y-y_c)^2)+2*x^2)+2*p1*x*y
% 	y_eqn = y_d == y*(1+k1*sqrt((x-x_c)^2+(y-y_c)^2)^2+k2*sqrt((x-x_c)^2+(y-y_c)^2)^4)+p1*(sqrt((x-x_c)^2+(y-y_c)^2)+2*x^2)+2*p2*x*y

% 	[x_u, y_u] = solve([x_eqn, y_eqn], [x, y]);
% end