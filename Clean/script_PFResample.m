    
    if (nParticles>1 & t>1)

        colours=lines(nRobots);

        if (plotStuff.pf)
            figure(plotStuff.pffig)
                set(gcf,'Visible',plotStuff.pfvisible);
                [val,ind]=max(weight(1,:));
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

        weight=weight/sum(weight);
        if (plotStuff.weight)
            figure(plotStuff.weightfig)
                plot(weight,'k-o')
                xlabel('Particles [n]');
                ylabel('Weight []')
        end
        
        [robOdom,robOdomReverse,map,robPoseMapFrame,robPoseMapFrameReverse,weight]=resample(robOdom,robOdomReverse,map,robPoseMapFrame,robPoseMapFrameReverse,weight);
        
    end