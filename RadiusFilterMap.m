% This function takes in the pointcloud map and applies a radius filter to
% remove isolated points
% Input - map (N x 3), radius(in m), num_neighb - Threshold for isolation
% Output - map (N x 3)
function [out_map] = RadiusFilterMap(map, radius, num_neighb)
    in_map_pc = pointCloud(map);
    temp_map = map;
    temp_map_pc = in_map_pc;
    out_map = [];
    for i=1:size(in_map_pc.Location, 1)
        point = in_map_pc.Location(i,:);
        [neighb, ~] = findNeighborsInRadius(temp_map_pc, point, radius);
%         [neighb, ~] = findNeighborsInRadius(in_map_pc, point, radius);
        % If number of neighbors is sufficient, keep point in map
        if(size(neighb,1) >= num_neighb)
            out_map = cat(1, out_map, point);
        else
            idx = find(ismember(temp_map, point, 'rows'));
            temp_map(idx,:) = [];
            temp_map_pc = pointCloud(temp_map);
        end
    end
end