function [newXRc, newXRa, newMap, newWeight] = resampleHoward(xRc, xRa, map, weight, nRobots)

N = length(weight); % Number of particles

% Create new pose arrays
newXRc = cell(nRobots,1);
newXRa = cell(nRobots,1);
for rob = 1:nRobots
    newXRc{rob} = zeros(size(xRc{rob})); % Allocate new sample memory
    newXRa{rob} = zeros(size(xRa{rob}));
end
newMap = zeros(size(map));
newWeight = repmat(1/N, size(weight)); % Set new weights to normalized uniform

% Recursively generate "CDF"
c = zeros(1,N); % Discrete weight CDF
c(1) = weight(1); % Base
for i = 2:N
   c(i) = c(i-1) + weight(i); 
end

% Sample from CDF based on initial threshold offset
u = zeros(1,N); %Threshold
u(:,1) = rand()/N; % First threshold

i = 1; % Reset marginal CDF location
for j = 1:N
    % Go through each marginal CDF to generate new sample
    while u(j) > c(i)
       i = i+1;       % Increment CDF index for each marginal
    end
    % Accept Update samples
    for rob = 1:nRobots
        newXRc{rob}(:,j) = xRc{rob}(:,i);
        newXRa{rob}(:,j) = xRa{rob}(:,i);
    end
    newMap(:,:,j) = map(:,:,i);
    u(j+1) = u(j) + 1/N; % Increment thresholds by constant
end


end
