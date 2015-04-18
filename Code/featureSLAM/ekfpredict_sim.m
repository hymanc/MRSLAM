function ekfpredict_sim(u)
% EKF-SLAM prediction for simulator process model

global Param;
global State;

% t-1 state and covariance
mu = State.Ekf.mu;
Sigma = State.Ekf.Sigma;
dr1 = u(1);
dt = u(2);

%% Mean Prediction
% Predict mean (update only robot pose)
State.Ekf.mu = prediction(State.Ekf.mu, u); % Use deterministic odometry model

%% Covariance Prediction

Fx = [eye(3),zeros(3,length(mu)-3)];
% Pose-motion Jacobian
G = eye(size(Sigma));
G(1,3) = -dt.*sin(mu(3)+dr1);
G(2,3) =  dt.*cos(mu(3)+dr1);

alpha = Param.alphas;
% Odometry process noise
M = [alpha(1)*u(1)^2+alpha(2)*u(2)^2,   0,              0;
     0,     alpha(3)*u(2)^2+alpha(4)*(u(1)^2+u(3)^2),   0;
     0,     0,            alpha(1)*u(1)^2+alpha(2)*u(2)^2];
% Process-input Jacobian 
V = [-u(2)*sin(mu(3)+u(1)), cos(mu(3)+u(1)), 0;
      u(2)*cos(mu(3)+u(1)), sin(mu(3)+u(1)), 0;
      1,                    0,               1]; 
% Adjusted pose-input noise  
R = V*M*V';

% Predict covariance
State.Ekf.Sigma = G*Sigma*G' + Fx'*R*Fx; 