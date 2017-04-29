% This function takes in the DSI (IxN), finds the local maxima of the
% ray density function, and returns the most confident pixels (IxN) 

function [depth_map] = GetClusters(KF_DSI)

C = 6;
filt_size = 5;
filter = fspecial('gaussian', filt_size); % Sigma to be determined

[confidence_map, pix_depth] = max(KF_DSI,[],3);

threshold_map = confidence_map > (imfilter(confidence_map, filter) - C);
depth_map = pix_depth .* threshold_map; % Zero depth for pixels with low confidence

end
