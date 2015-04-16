function plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t)
% plots the occupany map, current laser endpoints, robot, and trajectory

figure(1);
set(1, 'visible', 'off');

map = map';
imshow(ones(size(map)) - log_odds_to_prob(map))
axis ij equal tight;
%axis(mapBox);
s = size(map);
set(gcf, 'position', [50 50 s*5])
set(gca, 'position', [.05 .05 .9 .9])
hold on;

traj = [poses(1:t,1)';poses(1:t,2)'];
traj = world_to_map_coordinates(traj, gridSize, offset);
plot(traj(1,:),traj(2,:),'g');
plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',5,'linewidth',1);
plot(robPoseMapFrame(1),robPoseMapFrame(2),'b.','markersize',2,'linewidth',1);
plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'rs','markersize',1);

hold off;
filename = sprintf('plots/gridmap_%03d.png', t);
print(filename, '-dpng');

end
