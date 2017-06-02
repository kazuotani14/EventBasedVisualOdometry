function PlotResults(KF_dsi, depth_map, map)

colormap jet
IND = find(KF_dsi);
CNT = KF_dsi(IND);
[r,c,v] = ind2sub(size(KF_dsi),IND);
figure(1);
scatter3(c,r,v,10,CNT);
axis equal;

figure(2);
imagesc(depth_map);
axis equal;

figure(3);
scatter3(map(:,1), map(:,2), map(:,3), 2, 'filled');
axis equal;

end