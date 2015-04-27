%Check reverse odometry


%Workspace Setup
    addpath('Tools')
    addpath('COTs')
    
    more off
    %close all
    %clear all

%Load Data
    load('../CustomMapAndModel/CustomData1.mat')
    
    robotID=2;
    
    for a1=2:size(data(1).pose,2)
        newpose=ReverseOdometry(data(robotID).uact(:,a1-1),data(robotID).pose(:,a1),[0 0 0 0],OdometryModel);
        robDiffErr(:,a1-1)=newpose-data(robotID).pose(:,a1-1);
        robDiffErr(3,a1-1)=normalizeTheta(robDiffErr(3,a1-1));
    end
    
    
    
    figure(13421)
    for a1=1:3
        subplot(3,1,a1)
            plot(robDiffErr(a1,:),'k')
    end