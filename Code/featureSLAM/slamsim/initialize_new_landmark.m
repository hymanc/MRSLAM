function Li = initialize_new_landmark(z, R)
% z Measurement
% R measurement uncertainty?

global Param;
global State;

% Alter state mean
% Estimate new landmark location from measurement and predicted state
mu = State.Ekf.mu(State.Ekf.iR); % Estimated pose
newMu = mu(1:2,1)+[z(1)*cos(z(2)+mu(3)) ; z(1)*sin(z(2)+mu(3))]; % New landmark location
State.Ekf.mu = [State.Ekf.mu ; newMu]; % Append new landmark to state

% Initialize state covariance
newSigma = 1E14*eye(2); %TODO: Initialize covariance to something else?
State.Ekf.Sigma = blkdiag(State.Ekf.Sigma, newSigma);

N = State.Ekf.nL;
State.Ekf.iM = [State.Ekf.iM, [4+2*N, 5+2*N]];
State.Ekf.iL = [State.Ekf.iL, {[4+2*N, 5+2*N]}];

% if State.Ekf.nL == 0
%    State.Ekf.iM = [4,5]; % Initialize 1st landmark location after pose state
%    State.Ekf.iL = {[4,5]};
%else
%    maxIdx = max(State.Ekf.iM);
%    State.Ekf.iM = [State.Ekf.iM, [maxIdx+1, maxIdx+2]]; % Append list of map indices
%    State.Ekf.iL = [State.Ekf.iL, {[maxIdx+1, maxIdx+2]}]; % Append cell array version
%end
State.Ekf.sL = [State.Ekf.sL, z(3)]; % Add new signature to list
State.Ekf.nL = State.Ekf.nL + 1; % Increment landmark count

Li = length(State.Ekf.sL);
end
