fontsize=14;
fontname='Times';
nParticles=100;
nRobots=10;


figure(1)
    colour=lines(nRobots);
    combinedMap=sum(map(:,:,:,1),3);
    imagesc(1-log_odds_to_prob(combinedMap));
    colormap('gray');
    axis image;
    hold on
    for a2=1:nParticles
        for a1=1:nRobots 
            ind=robPoseMapFrame(1,:,a1,a2)~=0;
            plot(robPoseMapFrame(2,ind,a1,a2),robPoseMapFrame(1,ind,a1,a2),'Color',colour(a1,:));
        end
    end
    hold off;
    axis image;
    set(gca,'FontSize',fontsize,'FontName',fontname);
    xlabel('x [m]')
    ylabel('y [m]')
    print(gcf,'TheMAP.png','-dpng')
    
    
figure(2)
mkdir PF
    colour=lines(nRobots);
    for a2=1:nParticles
        combinedMap=sum(map(:,:,:,a2),3);
        imagesc(1-log_odds_to_prob(combinedMap));
        colormap('gray');
        axis image;
        pause(0.1)
        drawnow;
        set(gca,'FontSize',fontsize,'FontName',fontname);
        title(sprintf('Particle %d',a2));
        xlabel('x [m]')
        ylabel('y [m]')
        print(gcf,sprintf('PF/TheMAP_%03d.png',a2),'-dpng')
    end

    
    
    
    
    