%==== Open data file ====
fid = fopen('../data/shapes_rotation/events.txt');
% fid = fopen('../data/shapes_translation/events.txt');
%==== Reading New Events ====

% allocate space for the mat representation
event_mat = zeros(23126288,4); %shapes_rotation
% event_mat = zeros(17363976,4); %shapes_translation

tline = fgetl(fid);
tic
i = 1;
while ischar(tline)
	arr = str2num(tline);
	t = arr(1);
	x = arr(2);
	y = arr(3);
	polarity = arr(4);
	event_mat(i,:) = [t, x, y, polarity];
	i = i+1;
	if mod(i,1000) == 0
		i
	end
	% plot(x,y);
	% fprintf('Time %.5f - (%.5f, %.5f) \t polarity: %d\n', t, x, y, polarity)
	tline = fgetl(fid);
	% toc
end
save shapes_rotation_events.mat event_mat
% save shapes_translation_events.mat event_mat
toc