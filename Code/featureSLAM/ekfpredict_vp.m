function ekfpredict_vp(u,deltaT)
% EKF-SLAM prediction for Victoria Park process model

global Param;
global State;

% t-1 mean and covariance
mu = State.Ekf.mu;
Sigma = State.Ekf.Sigma;
pose = State.Ekf.mu(State.Ekf.iR);

%% Predict mean
a = Param.a; 
b = Param.b;
L = Param.L;

DeltaPose = [u(1)*cos(mu(3))-(u(1)/L)*tan(u(2))*(a*sin(mu(3))+b*cos(mu(3)));
             u(1)*sin(mu(3))+(u(1)/L)*tan(u(2))*(a*cos(mu(3))-b*sin(mu(3)));
            (u(1)/L)*tan(u(2))].*deltaT;
State.Ekf.mu(State.Ekf.iR) = pose + DeltaPose; % Only update pose portion
State.Ekf.mu(3) = minimizedAngle(State.Ekf.mu(3)); % Wrap angle

%% Predict covariance
% Motion-State Jacobian
%Gx = [1, 0,  -deltaT*(u(1)*sin(mu(3))+(u(1)/L)*tan(u(2))*(a*cos(mu(3)) - b*sin(mu(3))));
%     0, 1,   deltaT*(u(1)*cos(mu(3))-(u(1)/L)*tan(u(2))*(a*sin(mu(3)) + b*cos(mu(3))));
%     0, 0,   1];
G = eye(size(Sigma));
G(1,3) =  -deltaT*(u(1)*sin(mu(3))+(u(1)/L)*tan(u(2))*(a*cos(mu(3)) - b*sin(mu(3))));
G(2,3) =  deltaT*(u(1)*cos(mu(3))-(u(1)/L)*tan(u(2))*(a*sin(mu(3)) + b*cos(mu(3))));
% Motion input Jacobian
V = [cos(mu(3))-(1/L)*tan(u(2))*(a*sin(mu(3))+b*cos(mu(3))), (u(1)/L)*sec(u(2)).^2*(a*sin(mu(3))+b*cos(mu(3)));
     sin(mu(3))+(1/L)*tan(u(2))*(a*cos(mu(3))-b*sin(mu(3))), (u(1)/L)*sec(u(2)).^2*(a*cos(mu(3))-b*sin(mu(3)));
     (1/L)*tan(u(2))                                       , (u(1)/L)*sec(u(2)).^2];

Fx = [eye(3),zeros(3,length(State.Ekf.mu)-3)];
%Fu = [eye(2),zeros(2,length(State.Ekf.mu)-2)];

State.Ekf.Sigma = G*Sigma*G' + G*Fx'*Param.Qf*Fx*G' + Fx'*V*Param.Qu*V'*Fx;