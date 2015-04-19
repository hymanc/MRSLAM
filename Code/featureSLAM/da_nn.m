function [Li] = da_nn(z)
% perform nearest-neighbor data association

global Param;
global State;

Li = zeros(1,size(z,2));

% Loop over all observations
for i = 1:size(z,2)
    % Loop through all landmarks and compute minimum Mahalanobis distance
    mindex = 0;
    dmin = 1E14; 
    for j = 1:State.Ekf.nL
        landmarkRange = State.Ekf.iL{j};
        muR = State.Ekf.mu(State.Ekf.iR);
        obsLocation = State.Ekf.mu(1:2) + [z(1,i)*cos(muR(3)+z(2,i)); z(1,i)*sin(muR(3)+z(2,i))];
        knownMu = State.Ekf.mu(landmarkRange);
        knownSigma = State.Ekf.Sigma(landmarkRange, landmarkRange);
        d = ((knownMu-obsLocation)'/knownSigma)*(knownMu-obsLocation);
        %d = pdist([knownMu'; obsLocation'], 'mahalanobis', knownSigma); % Compute Mahalanobis distance to known landmark
        if((d < Param.NNThreshold) & (d < dmin))
           mindex = j;
           dmin = d; 
        end
    end
    Li(i) = mindex;
end