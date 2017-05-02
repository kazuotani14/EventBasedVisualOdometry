% This function takes in the depth map and applies a median filter with
% associated depth pixels
% Input - depth_map (w x h x 1)
% Output - depth_map (w x h x 1)
function [depth_map] = MedianFilterDepthMap(depth_map, half_window)
    [h, w] = size(depth_map);
    
    depth_interm = padarray(depth_map, half_window);
    for y=half_window(1)+1:half_window(1)+h
        for x = half_window(2)+1:half_window(2)+w
            piece = depth_interm(y-half_window(1):y+half_window(1), x-half_window(2):x+half_window(2));
            depth_map(y-half_window(1), x-half_window(2)) = median(piece(piece~=0));
        end
    end
    depth_map(isnan(depth_map)) = 0;
    depth_map = round(depth_map);
end