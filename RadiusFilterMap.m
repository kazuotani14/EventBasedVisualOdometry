% This function takes in the pointcloud map and applies a radius filter to
% remove isolated points
% Input - map (N x 3), radius(in m), num_neighb - Threshold for isolation
% Output - map (N x 3)
function [map] = RadiusFilterMap(map, radius, num_neighb)
    in_map_pc = pointCloud(map);
    out_map = [];
    for i=size(in_map_pc.Location, 1)
        point = in_map_pc.Location(i,:);
        [neighb, ~] = findNeighborsInRadius(in_map_pc, point, radius);
        % If number of neighbors is sufficient, keep point in map
        if(size(neighb,1) > num_neighb)
            cat(1, out_map, point);
        end
    end
end