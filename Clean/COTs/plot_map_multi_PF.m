function plot_map_multi_PF(map, mapBox, robPoseMapFrame, data, laserEndPntsMapFrame, gridSize, offset, t)
% plots the occupany map, current laser endpoints, robot, and trajectory

figure(1);
set(1, 'visible', 'off');

map = map';
imshow(ones(size(map)) - log_odds_to_prob(map))
axis ij equal tight;
%axis(mapBox);
s = size(map);
%set(gcf, 'position', [50 50 s*5])
set(gcf, 'position', [50 50 s])
set(gca, 'position', [.05 .05 .9 .9])
hold on;

colours=lines(numel(data));
for a1=1:numel(data)
    traj = data(a1).pose(1:2,1:t);
    traj = world_to_map_coordinates(traj, gridSize, offset);
    plot(traj(1,:),traj(2,:),'Color',colours(a1,:));
    plot(robPoseMapFrame(1,a1),robPoseMapFrame(2,a1),'o','markersize',5,'linewidth',1,'Color',colours(a1,:));
    plot(robPoseMapFrame(1,a1),robPoseMapFrame(2,a1),'.','markersize',2,'linewidth',1,'Color',colours(a1,:));
    plot(laserEndPntsMapFrame{a1}(1,:),laserEndPntsMapFrame{a1}(2,:),'rs','markersize',1);
end
hold off;
filename = sprintf('plots/gridmap_%03d.png', t);

print(gcf,filename, '-dpng');

end
