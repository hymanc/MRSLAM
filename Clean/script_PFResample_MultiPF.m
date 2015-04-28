    
    if (nParticles>1 & t>1)
    %if ( t>1)

        if (plotStuff.pf)
            figure(plotStuff.pffig)
                set(gcf,'Visible',plotStuff.pfvisible);
                [val,ind]=max(weight(:,1));
                %imagesc(1-log_odds_to_prob(sum(map,3)));
                imagesc(1-log_odds_to_prob(sum(map(:,:,ind),3)));
                set(gca,'FontSize',plotStuff.fontsize,'FontName',plotStuff.fontname);
                axis image;
                colormap('gray');
                hold on;
                for a1=1:nRobots
                    inds=1:min([t maxT]);
                    revinds=t:-1:(revcounters(a1));
                    for a2=1:nParticles
                        plot(robPoseMapFrame(2,inds,a1,a2),robPoseMapFrame(1,inds,a1,a2),'Color',colours(a1,:))
                        plot(robPoseMapFrameReverse(2,revinds,a1,a2),robPoseMapFrameReverse(1,revinds,a1,a2),'--','Color',colours(a1,:))
                    end
                    
                    plot((squeeze(robOdom(2,a1,:))-offset(2))/gridSize,(squeeze(robOdom(1,a1,:))-offset(1))/gridSize,'.','Color',colours(a1,:))
                    plot((squeeze(robOdomReverse(2,a1,:))-offset(2))/gridSize,(squeeze(robOdomReverse(1,a1,:))-offset(1))/gridSize,'.','Color',colours(a1,:))
                    
                    plot((data(robotInds(a1)).pose(2,inds(end))-offset(2))/gridSize,(data(robotInds(a1)).pose(1,inds(end))-offset(1))/gridSize,'o','Color',colours(a1,:))
                    plot((data(robotInds(a1)).pose(2,revinds(end))-offset(2))/gridSize,(data(robotInds(a1)).pose(1,revinds(end))-offset(1))/gridSize,'s','Color',colours(a1,:))
                end
                
                hold off;
                drawnow;
                print(gcf,sprintf('plots/PFBest-%03d.png',t),'-dpng');
        end

        for a1=1:nRobots
            weight(:,a1)=weight(:,a1)/sum(weight(:,a1));
            weightReverse(:,a1)=weightReverse(:,a1)/sum(weightReverse(:,a1));
        end
        
        if (plotStuff.weight)        
            figure(plotStuff.weightfig)
            for a1=1:nRobots
                plot(weight,'-o','Color',colours(a1,:))
                hold on
                plot(weightReverse,'--o','Color',colours(a1,:))
            end
            xlabel('Particles [n]');
            ylabel('Weight []')
            hold off;
        end

        for a1=1:nRobots
            [robOdom(:,a1,:),map,robPoseMapFrame(:,:,a1,:),weight(:,a1)]=resampleSingle(robOdom(:,a1,:),map,robPoseMapFrame(:,:,a1,:),weight(:,a1));
            [robOdomReverse(:,a1,:),~,robPoseMapFrameReverse(:,:,a1,:),weightReverse(:,a1)]=resampleSingle(robOdomReverse(:,a1,:),map,robPoseMapFrameReverse(:,:,a1,:),weightReverse(:,a1));
        end
        
    end