

cost=inf;
c1=1;
lims=[0.04925 0.05];
c1=1;
while (cost>1E-6 & c1<20)
    dt=mean(lims);
    cost1=CompareOnboardOdometry(lims(1));
    cost2=CompareOnboardOdometry(lims(2));
    cost=CompareOnboardOdometry(dt);
    if (cost2>cost)
        lims(2)=dt;
    else
        lims(1)=dt;
    end
    fprintf('[%f %f]\n',lims);
    c1=c1+1;
end

fprintf('Optimal dt: %0.15f\n',dt);