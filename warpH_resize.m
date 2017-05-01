function warp_im = warpH_resize(im, H, out_size, scaling)
%function warp_im = warpH(im, H, out_size,fill_value)
% warpH projective image warping
%   warp_im=warpA(im, A, out_size)
%   Warps a size (w,h,channels) image im using the  (3,3) homography H
%   producing an (out_size(1),out_size(2)) output image warp_im
%
%   warped_coords := H*source_coords
%
%   warp_im=warpH(im, A, out_size,fill_value)
%   Uses fill_value (scalar) to paint empty regions.

fill_value = 0;

% tform = maketform( 'projective', H'); 
% % warp_im = imtransform( im, tform, 'bilinear', 'XData', ...
% % 	[1 out_size(2)], 'YData', [1 out_size(1)], 'Size', out_size(1:2), 'FillValues', fill_value*ones(size(im,3),1));
% warp_im = imtransform( im, tform, 'bilinear');

tform = projective2d(H');
warp_im = imwarp(im, tform);

des_size = out_size .* scaling;
warp_size = size(warp_im);
x0 = max(1, floor(warp_size(1)/2 - des_size(1)/2));
x1 = min(warp_size(1), floor(warp_size(1)/2 + des_size(1)/2));
y0 = max(1, floor(warp_size(2)/2 - des_size(2)/2));
y1 = min(warp_size(2), floor(warp_size(2)/2 + des_size(2)/2));
disp([x1-x0, y1-y0]);
warp_im = warp_im(x0:x1, y0:y1);
warp_im = imresize(warp_im, out_size);

