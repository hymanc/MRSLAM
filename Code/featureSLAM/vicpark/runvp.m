function runvp(nSteps,pauseLen,makeVideo)

global Param;
global State;
global Data;

if ~exist('nSteps','var') || isempty(nSteps)
    nSteps = inf;
end

if ~exist('pauseLen','var')
    pauseLen = 0; % seconds
end

% Attempt to open video file
%if makeVideo
%    try
%        votype = 'VideoWriter';
%        vo = VideoWriter('video_vicpark', 'MPEG-4');
%        set(vo, 'FrameRate', min(5, 1/pauseLen));
%        open(vo);
%    catch
%        error('Failure to write video')
%    end
%end

Data = load_vp_si();

% Initalize Params
%===================================================
% vehicle geometry
Param.a = 3.78; % [m]
Param.b = 0.50; % [m]
Param.L = 2.83; % [m]
Param.H = 0.76; % [m]

% 2x2 process noise on control input
sigma.vc = 0.02; % [m/s]
sigma.alpha = 2*pi/180; % [rad]
Param.Qu = diag([sigma.vc, sigma.alpha].^2);

% 3x3 process noise on model error
sigma.x = 0.1; % [m]
sigma.y = 0.1; % [m]
sigma.phi = 0.5*pi/180; % [rad]
Param.Qf = diag([sigma.x, sigma.y, sigma.phi].^2);

% 2x2 observation noise
sigma.r = 0.05; % [m]
sigma.beta = 1*pi/180; % [rad]
Param.R = diag([sigma.r, sigma.beta].^2);

Param.NNThreshold = 100; % Nearest neighbor Mahalanobis distance threshold
%===================================================

% Initialize State
%===================================================
State.Ekf.mu = [Data.Gps.x(2), Data.Gps.y(2), 36*pi/180]';
State.Ekf.Sigma = zeros(3);

State.Pf.mu = [Data.Gps.x(2), Data.Gps.y(2), 36*pi/180];


global AAr;
AAr = (0:360)*pi/360;

figure(1); clf;
axis equal;

ci = 1; % control index
t = min(Data.Laser.time(1), Data.Control.time(1));
predictTimes = []; % Prediction performance profiling data
updateTimes = [];  % Update performance profiling data

for k=1:min(nSteps, length(Data.Laser.time))
    kstr = sprintf('%d',k);
    disp kstr
    while (Data.Control.time(ci) < Data.Laser.time(k))
       % control available
       dt = Data.Control.time(ci) - t;
       t = Data.Control.time(ci);
       u = [Data.Control.ve(ci), Data.Control.alpha(ci)]';
       tic; % Start prediction timer
       ekfpredict_vp(u, dt); % Update for control input
       etime = toc; % Stop prediction timer
       predictTimes = [predictTimes, [t;etime;State.Ekf.nL]];
       ci = ci+1;
    end
    
    % observation available
    dt = Data.Laser.time(k) - t;
    t = Data.Laser.time(k);
    z = detectTreesI16(Data.Laser.ranges(k,:));
    
    zcorr = z-repmat([0;pi/2;0],1,size(z,2)); % Offset bearing measurement by pi/2 to use same model in ekfUpdate()
    tic; % Start timer
    ekfupdate(zcorr); % Run EKF sensor update, alter angle
    etime = toc; % Stop timer
    updateTimes = [updateTimes, [t;etime;State.Ekf.nL]]; % Stop timer
    doGraphics(z); % Update plot graphics
    % Write video frame
    if makeVideo
        F = getframe(1);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
    
    drawnow; % Update plots
    if pauseLen > 0
        pause(pauseLen);
    end
end

% 3.3 Plots
% Plots against time
figure(2);
subplot(3,1,1);
plot(predictTimes(1,:),predictTimes(2,:),'b');
xlabel('Simulation Time (s)');
ylabel('Computation Time (s)');
title('EKF Prediction Computation Time');
subplot(3,1,2);
plot(updateTimes(1,:),updateTimes(2,:),'r');
xlabel('Simulation Time (s)');
ylabel('Computation Time (s)');
title('EKF Update Computation Time');
subplot(3,1,3);
plot(updateTimes(1,:),updateTimes(3,:));
xlabel('Simulation Time (s)');
ylabel('Number of Landmarks');
title('Size of Map');

% Write video
if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end
%% End of Function

%==========================================================================
function doGraphics(z)
% Put whatever graphics you want here for visualization
%
% WARNING: this slows down your process time, so use sparingly when trying
% to crunch the whole data set!

global Param;
global State;

% plot the robot and 3-sigma covariance ellipsoid
plotbot(State.Ekf.mu(1), State.Ekf.mu(2), State.Ekf.mu(3), 'black', 1, 'blue', 1);
hold on;

plotcov2d( State.Ekf.mu(1), State.Ekf.mu(2), State.Ekf.Sigma, 'blue', 0, 'blue', 0, 3);

% restrict view to a bounding box around the current pose
%BB=20;
%axis([[-BB,BB]+State.Ekf.mu(1), [-BB,BB]+State.Ekf.mu(2)]);

axis equal;

% project raw sensor detections in global frame using estimate pose
xr = State.Ekf.mu(1);
yr = State.Ekf.mu(2);
tr = State.Ekf.mu(3);
for k=1:size(z,2)
    r = z(1,k);
    b = z(2,k);
    xl = xr + r*cos(b+tr-pi/2);
    yl = yr + r*sin(b+tr-pi/2);
    plot([xr; xl], [yr; yl],'r',xl,yl,'r*');
end

%% Plot Landmark covariances
%muPose = State.Ekf.mu(State.Ekf.iR);
%SigmaPose = State.Ekf.Sigma(State.Ekf.iR,State.Ekf.iR);
% Landmarks
for i = 1:State.Ekf.nL
    % Check if landmark is near visible region, otherwise do not plot
    idx = State.Ekf.iL{i};
    lmu = State.Ekf.mu(idx);
    cov = State.Ekf.Sigma(idx,idx); 
    plotcov2d(lmu(1), lmu(2), cov, 'g', 0, 'r', 0, 3);
end

hold off;

