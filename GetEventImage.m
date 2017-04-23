% This function should give an event image based off of the start time of collecting
% events. As specified, they wait till the number of points is a percentage of the
% map, and not more than one event hits a pixel. We do the latter

function [event_image, end_time] = GetEventImage(start_time, event_mat)
event_image = zeros(180,240); %this is the size of the camera
event_idx = find(event_mat(:,1) >= start_time,1);

%build event image while no pixel gets 2 events
while max(event_image) < 2;
	%the plus ones are to deal with the fact that normal people zero index
	event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) = ...
		event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) + 1;
	event_idx = event_idx + 1;
end

%clean up the 2 event pixel
event_idx = event_idx - 1;
event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) = ...
	event_image(event_mat(event_idx,3)+1,event_mat(event_idx,2)+1) - 1;
end_time = event_mat(event_idx+1,1);

end