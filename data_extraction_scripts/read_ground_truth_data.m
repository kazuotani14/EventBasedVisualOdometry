%==== Open data file ====
% fid = fopen('../data/shapes_rotation/groundtruth.txt');
fid = fopen('../data/shapes_translation/groundtruth.txt');

%==== Reading New Events ====

% allocate space for the mat representation
% groundtruth_mat = zeros(11883,8); %shapes_rotation
groundtruth_mat = zeros(11947,8); %shapes_translation

tline = fgetl(fid);
tic
i = 1;
while ischar(tline)
	arr = str2num(tline);
	t = arr(1);
	x = arr(2);
	y = arr(3);
	z = arr(4);
	q_x = arr(5);
	q_y = arr(6);
	q_z = arr(7);
	q_w = arr(8);
	groundtruth_mat(i,:) = [t, x, y, z, q_x, q_y, q_z, q_w];
	i = i+1;
	if mod(i,1000) == 0
		i
	end
	% plot(x,y);
	% fprintf('Time %.5f - (%.5f, %.5f) \t polarity: %d\n', t, x, y, polarity)
	tline = fgetl(fid);
	% toc
end
% save shapes_rotation_groundtruth.mat groundtruth_mat
save shapes_translation_groundtruth.mat groundtruth_mat
toc