function warp_im = warpH_rect(im, H, out_size, scaling)
% Assume im is zeros where there are no points
nonzeros = find(im);

n_points = length(nonzeros);
points = zeros(n_points, 3);
for i = 1:n_points
    [x, y] = ind2sub(size(im), nonzeros(i));
    points(i,:) = [x, y, 1];
end

warped_points = points*H;
warp_size = floor(scaling.*out_size);
warp_im = zeros(warp_size);
for i = 1:n_points
    coord = warped_points(i,:);
    x = floor(coord(1)); y = floor(coord(2));
    if (x>0 && x<scaling(1)*out_size(1) && y>0 && y<scaling(2)*out_size(2))
        warp_im(x,y) = 1;
    end
end

% TODO check this
warp_im = imresize(warp_im, out_size);

end