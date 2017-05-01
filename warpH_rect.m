function warp_im = warpH_rect(im, H, out_size, scaling)
% Assume im is zeros where there are no points
nonzeros = find(im);

% 
n_points = length(nonzeros);
points = zeros(n_points, 3);
for i = 1:n_points
    [x, y] = ind2sub(size(im), nonzeros(i));
    points(i,:) = [x, y, 1];
end

warped_points = points*H;
scale = [scaling(1,1), scaling(2,2)];
warp_im = zeros(out_size);
for i = 1:n_points
    coord = warped_points(i,:);
    x = floor(coord(1)); y = floor(coord(2));
    if (x>0 && x<scale(1)*out_size(1) && y>0 && y<scale(2)*out_size(2))
        x = floor(x/scale(1));
        y = floor(y/scale(2));
        warp_im(x,y) = 1;
    end
end

% TODO check this
warp_im = imresize(warp_im, out_size);

end