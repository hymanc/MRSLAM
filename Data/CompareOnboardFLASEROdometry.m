function cost=CompareOnboardFLASEROdometry(dt)

    fontsize=14;
    fontname='Times';

    load('albert.mat')

    if (nargin~=1)
        dt=0.049613281248952;
    end
    
    figure(1)
        plot(odomodom(1,:),odomodom(2,:),'b');
        hold on;
        plot(odom(1,:),odom(2,:),'r');
    
        
    odomest=odom(:,1);
    Q=diag([100 100 1]);
    cost=(odomest(:,1)-odom(:,1))'*Q*(odomest(:,1)-odom(:,1));
    for a1=2:size(odom,2)
        %odomest=[odomest Odometry(inputsodom(1,a1),inputsodom(2,a1),todom(a1)-todom(a1-1),odomest(:,end))];
        odomest=[odomest Odometry(inputsodom(1,a1),inputsodom(2,a1),dt,odomest(:,end))];
        cost=cost+(odomest(:,a1)-odomodom(:,a1))'*Q*(odomest(:,a1)-odomodom(:,a1));
    end

    plot(odomest(1,:),odomest(2,:),'k');
    set(gca,'FontSize',fontsize,'FontName',fontname);
    xlabel('Time')
    ylabel('y [m]')
    hold off;
    print(gcf,'XYOdometry.png','-dpng');
    
    figure(2)
        plot(todom,odomodom(3,:),'b');
        hold on;
        plot(t,odom(3,:),'r');
        plot(todom,odomest(3,:),'k');
        set(gca,'FontSize',fontsize,'FontName',fontname);
        xlabel('Time')
        ylabel('\theta [rad]')
        hold off;
        print(gcf,'thetaOdometry.png','-dpng');
        
    figure(3)
        plot(todom,odomodom(1,:),'b');
        hold on;
        plot(t,odom(1,:),'r');
        plot(todom,odomest(1,:),'k');
        set(gca,'FontSize',fontsize,'FontName',fontname);
        xlabel('Time')
        ylabel('x [m]')
        hold off;
        print(gcf,'XOdometry.png','-dpng');
        
    figure(4)
        plot(todom,odomodom(2,:),'b');
        hold on;
        plot(t,odom(2,:),'r');
        plot(todom,odomest(2,:),'k');
        set(gca,'FontSize',fontsize,'FontName',fontname);
        xlabel('Time')
        ylabel('y [m]')
        hold off;
        print(gcf,'YOdometry.png','-dpng');
        
        
   
end