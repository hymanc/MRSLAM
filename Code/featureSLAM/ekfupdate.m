function ekfupdate(z)
% EKF-SLAM update step for both simulator and Victoria Park data set

global Param;
global State;

% returns state vector indices pairing observations with landmarks
switch lower(Param.dataAssociation)
    case 'known'
        Li = da_known(z(3,:));
    case 'nn'
        Li = da_nn(z(1:2,:));
    case 'jcbb'
        Li = da_jcbb(z(1:2,:), Param.R);
    otherwise
        error('unrecognized data association method: "%s"', Param.dataAssociation);
end

%% Perform update
if(strcmp(Param.updateMethod,'batch')) % Check for update method (batch/seq)
    %% Batch update
    % Initialize unseen landmarks according to data association
    for i = 1:size(Li,2)
        if(Li(i) == 0)
            Li(i) = initialize_new_landmark(z(:,i),Param.R);
        end
    end
    mu = State.Ekf.mu;
    Sigma = State.Ekf.Sigma;
    iR = State.Ekf.iR; % Robot indices
    % Iterate over landmarks to generate stacked/batch update
    H = [];
    zdiff = [];
    Q = [];
    for i = 1:size(Li,2)
        j = Li(i);
        landmark = State.Ekf.mu(State.Ekf.iL{j});
        % Generate stacked expected measurement
        [zhat,delta, q] = expectedMeasurement(mu(iR), landmark);
        %zdiff(2*i:2*i+1,1) = z(1:2,i)-zhat;
        if(size(zdiff,1)==0)
            zdiff = z(1:2,i)-zhat;
        else
            zdiff = [zdiff ; (z(1:2,i)-zhat)];
        end
        % Compute estimation error
        sq = sqrt(q);
        dx = delta(1);
        dy = delta(2);
        % Generate stacked measurement Jacobian
        N = State.Ekf.nL;   % Number of landmarks
        Fxj = [[eye(3);zeros(2,3)], zeros(5,2*j-2), [zeros(3,2);eye(2)],zeros(5,2*N-2*j)];
        Hxj = (1/q).*[-sq*dx, -sq*dy, 0, sq*dx, sq*dy;    
                                    dy,     -dx,   -q, -dy,   dx]*Fxj;   
        if(size(H,1) == 0)
            H = Hxj;     % Initialize measurement Jacobian
        else
            H = [H;Hxj]; % Stack measurement Jacobian
        end
        Q = blkdiag(Q,Param.R);
    end
    S = H*Sigma*H'+Q; % Batch innovation
    K = Sigma*H'/S;         % Batch Kalman gain
    % Compute mean update
    State.Ekf.mu = State.Ekf.mu+K*(zdiff);  
    State.Ekf.mu(3) = minimizedAngle(State.Ekf.mu(3)); % Wrap angle
    % Compute covariance update using Joseph form Kalman update
    Joe = eye(size(Sigma))-K*H;
    State.Ekf.Sigma = Joe*Sigma*Joe'+K*Q*K';
else
    %% Sequential update
    % Loop over observed landmarks (post data-association)
    for i = 1:length(Li)
        j = Li(i); % Current landmark index in state
        if(j == 0) % If landmark was first observed, create new
            j = initialize_new_landmark(z(:,i),Param.R); % Crete new landmark and update index
        end
        landmark = State.Ekf.mu(State.Ekf.iL{j}); % Landmark est. location
        mu = State.Ekf.mu(State.Ekf.iR);          % Robot pose
        [zhat, delta, q] = expectedMeasurement(mu, landmark); % Compute expected measurement (zhat)
        
        % Compute Measurement Jacobian
        N = State.Ekf.nL;   % Number of landmarks
        Fxj = [[eye(3);zeros(2,3)], zeros(5,2*j-2), [zeros(3,2);eye(2)],zeros(5,2*N-2*j)];
        sq = sqrt(q);
        dx = delta(1);
        dy = delta(2);
        H = (1/q).*[-sq*dx, -sq*dy, 0, sq*dx, sq*dy;    % Measurement Jacobian
                    dy,     -dx,   -q, -dy,   dx]*Fxj;
        Sigma = State.Ekf.Sigma;
        K = Sigma*H'/(H*Sigma*H'+Param.R); % Compute Kalman gain
        % Kalman update
        State.Ekf.mu = State.Ekf.mu + K*(z(1:2,i)-zhat);
        State.Ekf.mu(3) = minimizedAngle(State.Ekf.mu(3)); % Wrap pose angle
        % Joseph form Kalman covariance update
        Joe = eye(size(Sigma))-K*H;
        State.Ekf.Sigma = Joe*Sigma*Joe' + K*Param.R*K';
    end

end


% 2.2 Plot associations
% if(strcmp(Param.dataAssociation,'nn'))
%     figure(3);
%     plot(); % TODO: store data assocation
%     title('Data Associations');
%     xlabel('Measurement #');
%     ylabel('Associated Landmark');
%     legend('Ground Truth', 'Nearest Neighbor Association');
% end
% End of Function

end
% 1.4 Dissayanake
% Plot x-
% Indices


function [z, delta, q] = expectedMeasurement(muPose, muLandmark)
    delta = [muLandmark(1)-muPose(1) ; muLandmark(2)-muPose(2)];
    q = delta'*delta;
    z = [sqrt(q); minimizedAngle(atan2(delta(2),delta(1))-muPose(3))];  % Expected measurement
end