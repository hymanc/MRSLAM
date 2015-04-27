function points = robotlaser_as_cartesian(scan, maxRange, subsample)

% set default params
if isempty(maxRange)
    maxRange = 15;
end
if isempty(subsample)
    subsample = false;
end

    %Specific to albert data set! - riboch
    rl.ranges=scan';
    rl.start_angle=-pi/2;
    rl.maximum_range=maxRange;
    rl.angular_resolution=pi/180;
    rl.laser_offset=[0 0 0];
    

numBeams = length(rl.ranges);
maxRange=min(maxRange, rl.maximum_range);
% apply the max range
idx = rl.ranges<maxRange & rl.ranges>0;

if (subsample)
    idx(2:2:end) = 0;
end

angles = linspace(rl.start_angle, rl.start_angle + numBeams*rl.angular_resolution, numBeams);
angles = angles(idx);
points = [rl.ranges(idx) .* cos(angles); rl.ranges(idx) .* sin(angles); ones(1, length(angles))];
transf = v2t(rl.laser_offset);

% apply the laser offset
points = transf * points;

end
