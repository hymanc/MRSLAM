function Li = da_known(z)
% EKF-SLAM data association with known correspondences

global Param;
global State;

Li = zeros(1,length(z));
% Find each landmark in list of previously known landmarks
for i = 1:length(z)
    exists = find(State.Ekf.sL == z(i));
    if(exists)
        Li(i) = exists; % Get indices of map landmark
    else % Create flag unseen landmark
        Li(i) = 0;
    end
end
