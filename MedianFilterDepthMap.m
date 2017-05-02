% This function takes in the depth map and applies a median filter with
% associated depth pixels
% Input - depth_map (w x h x 1)
% Output - depth_map (w x h x 1)
function [depth_map] = MedianFilterDepthMap(depth_map)
    window = [7 7];

    [h, w] = size(depth_map);
    
    depth_interm = padarray(depth_map, window);
    for y=window(1)+1:window(1)+h
        for x = window(2)+1:window(2)+w
            piece = depth_interm(y-window(1):y+window(1), x-window(2):x+window(2));
            depth_map(y-window(1), x-window(2)) = median(piece(piece~=0));
        end
    end
    
end