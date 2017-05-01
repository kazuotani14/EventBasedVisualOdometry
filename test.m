clear
clc
clf

event_image = imread('square.png');
event_image = ~imbinarize(rgb2gray(event_image));
load('shapes_translation_calib.mat')

event_image = imread('event_image1.png');

min_depth = 0.05;
max_depth = 1.5;
KF_depths = linspace(min_depth,max_depth,100);

KF_DSI = zeros(size(event_image,1),size(event_image,2),length(KF_depths));


%%

n_planes = size(KF_DSI,3);

Fx = calib.fx * 18.5 * 1e-6; Fy = calib.fy * 18.5 * 1e-6;
% cx = calib.cx; cy = calib.cy;
cx = 27.5; cy = 16.5;
fx = calib.fx; fy = calib.fy;
K = [fx, 0, cx;
    0, fy, cy;
    0, 0, 1];

t = [0.001, 0, 0]';
ang = [0, pi/4, 0];
R = eye(3);%eul2rotm(ang);

T_i = [R, t; 0,0,0,1];
T_kf = eye(4);


%%

T_kf2i = T_i;
T_i2kf = inv(T_kf2i);

H_i = K * [eye(3),zeros(3,1)] * T_i;
H_kf = K * [eye(3),zeros(3,1)] * T_kf;

H_i2kf = H_kf / H_i;

R = T_kf2i(1:3,1:3);
t = T_kf2i(1:3,4);

Cx = -R(:,1)'*t;
Cy = -R(:,2)'*t;
Cz = -R(:,3)'*t;

K = K * 18.5*1e-6;
K(3,3)=1;
H0 = K*[R(:,1), R(:,2), KF_depths(1)*R(:,3)+t];

% H0 = H_kf / H_i;
z0 = KF_depths(1);
for i=1:n_planes
    delta = (KF_depths(i) - Cz) / (z0 - Cz);
    H_kf2z = [delta, 0, (1-delta)*Cx; 0, delta, (1-delta)*Cy; 0, 0, 1];
    H{i} = H_kf2z*H0;
%     H{i} = H{i}/H{i}(3,3);
%     H{i} = H0;
%     H{i}(3,3) = KF_depths(i);
end

imref_obj = imref2d([size(KF_DSI,1),size(KF_DSI,2)]);

for i=1:n_planes
%    event_im_KF = warpH(event_image, H{i}, [size(KF_DSI,1),size(KF_DSI,2)]);

    tform = projective2d(H{i});
   event_im_KF = imwarp(event_image, tform);
   event_im_KF = imresize(event_im_KF, [size(event_image,1),size(event_image,2)]);
   
%    KF_DSI(:,:,i) = KF_DSI(:,:,i) + event_im_KF;
   imshow(event_im_KF);
%    spy(sparse(KF_DSI(:,:,i)));
   disp('update DSI')
%    disp(sum(sum(KF_DSI(:,:,i))));
   drawnow;
   pause(0.1)
end


%%

% KF_homographies = cell(n_planes,1);
% KF_Homographies = makeKfHomographies(n_planes, event_image, KF_depths, fx, fy);
%  
% H_i = K * [eye(3),zeros(3,1)] * T_i;
% H_kf = K * [eye(3),zeros(3,1)] * T_kf;
%  
% H_i2kf = H_kf / H_i;
%  
% kf_R = T_kf(1:3,1:3);
% i_R  = T_i(1:3, 1:3);
% kf_T = T_kf(1:3, 4);
% i_T  = T_i(1:3, 4);
% R_i2kf = kf_R*i_R';
% % R_i2kf = kf_R'*i_R;
%  
% T_i2kf = -kf_T + i_T;
% T_kis = [R_i2kf, T_i2kf; 0 0 0 1];
%  
% R_kf2i = R_i2kf';
% T_kf2i = -T_i2kf;
%  
% r1 = R_kf2i(:,1);
% r2 = R_kf2i(:,2);
% r3 = R_kf2i(:,3);
% t = T_kf2i;
%  
% Cx = -dot(r1,t);
% Cy = -dot(r2,t);
% Cz = -dot(r3,t);
%  
% H_i2z0 = [R_kf2i(:,1), R_kf2i(:,2), KF_depths(1)*R_kf2i(:,3)+T_kf2i];
% H_kf2z0 = inv(H_i2z0)*(H_i2kf);
% H_kf2zo(3,3) = 1;
%  
% % p_i = [52,81,1]';
% % p_kf = H_i2kf*p_i;
% 
%  
% H = cell(n_planes,1);
%  
% H_kf2z0 = eye(3); % This is what we need to determine!!
%  
% for i=1:n_planes
%     zi = KF_depths(i);
% %     H{i} = KF_Homographies{i}*H_kf2z1*H_i2kf;
%     % Trying dilation
%     delta = (zi-Cz) / (KF_depths(1)-Cz);
%     dilation = [delta, 0 , (1-delta)*Cx;
%                 0, delta, (1-delta)*Cy;
%                 0, 0, 1];
%     H{i} = dilation * H_i2z0;  % H_kf2z0 * H_i2kf; 
% end
%  
% for i=1:n_planes
% %    tform = projective2d(H{i}');
% %    event_im_KF = imwarp(event_image, tform, 'OutputView',imref_obj);
% %    event_im_KF = warpH_rect(event_image, H{i}, [size(KF_DSI,1),size(KF_DSI,2)], KF_Homographies{i});
%     event_im_KF = warpH(event_image, H{i}, [size(KF_DSI,1),size(KF_DSI,2)]);
% % increment relevant DSI voxels
%    KF_DSI(:,:,i) = KF_DSI(:,:,i) + event_im_KF;
%    imshow(event_im_KF)
%    drawnow;
% %    spy(sparse(KF_DSI(:,:,i)));
%    disp('update DSI')
% %    disp(sum(sum(KF_DSI(:,:,i))));
% %    pause(0.1)
% end

