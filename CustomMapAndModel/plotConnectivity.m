function plotConnectivity(MAP,mapEncounters,data)

    fontsize=14;
    fontname='Times';

    %close all

    if (nargin==0)
        load('CustomData1.mat')
        [MAP,PIXDIM]=getTheMAP(THEIMAGE);
    end

    z=exp(2*pi*1i*((1:numel(data))-1)/numel(data));
    figure(102)
        plot(real(z),imag(z),'ko');
        axis off;
        axis image;
        hold on;
        set(gca,'FontSize',fontsize,'FontName',fontname);
        for a1=1:numel(data)
            text(real(z(a1)+0.1*exp(2*pi*1i*(a1-1)/numel(data))),imag(z(a1)+0.05*exp(2*pi*1i*(a1-1)/numel(data))),sprintf('%d',a1));
        end
        for a1=1:size(mapEncounters,1);
            plot(real(z(mapEncounters(a1,2:3))),imag(z(mapEncounters(a1,2:3))),'k-');
        end
        hold off;
        
    
    figure(123)
    nm=[2*floor(numel(data)/4) min([4*floor(numel(data)/4)+1 ceil(numel(data)/2)])];
    for a2=1:numel(data)
        from=parseEncounters(mapEncounters,a2);
        subplot(nm(1),nm(2),a2)
        plot(real(z),imag(z),'ko');
        axis off;
        axis image;
        hold on;
        set(gca,'FontSize',fontsize,'FontName',fontname);
        for a1=1:numel(data)
            text(real(z(a1)+0.1*exp(2*pi*1i*(a1-1)/numel(data))),imag(z(a1)+0.05*exp(2*pi*1i*(a1-1)/numel(data))),sprintf('%d',a1));
        end
        for a1=1:size(from,1);
            plot(real(z(from(a1,2:3))),imag(z(from(a1,2:3))),'k-');
        end
        hold off;
        title(sprintf('Starting with robot %d.',a2))
    end
        
    
        
    figure(103)
        colours=lines(numel(data));
        imagesc(1-MAP)
        axis image;
        axis off
        colormap gray;
        hold on;
        for a2=1:numel(data)
            plot(data(a2).pose(2,:),data(a2).pose(1,:),'Color',colours(a2,:))
            hold on;
        end
        
        for a1=1:size(mapEncounters)
            x=[data(mapEncounters(a1,2)).pose(1,mapEncounters(a1,1)) data(mapEncounters(a1,3)).pose(1,mapEncounters(a1,1))];
            y=[data(mapEncounters(a1,2)).pose(2,mapEncounters(a1,1)) data(mapEncounters(a1,3)).pose(2,mapEncounters(a1,1))];
            plot(y,x,'--','Color',[0.5 0.5 0.5])
        end
        hold off;
        print(gcf,sprintf('Encounters.png'),'-dpng');
        cropBackground(sprintf('Encounters.png'));
        print(gcf,sprintf('Encounters.eps'),'-depsc');
        
        
    figure(105)
        plot([mapEncounters(1,1) mapEncounters(1,1)],[mapEncounters(1,2) mapEncounters(1,3)],'k');
        hold on;
        for a1=2:size(mapEncounters,1)
            plot([mapEncounters(a1,1) mapEncounters(a1,1)],[mapEncounters(a1,2) mapEncounters(a1,3)],'k');
        end
        hold off;
        set(gca,'FontSize',fontsize,'FontName',fontname);
        xlabel('Time [s]')
        ylabel('Encounter')
        print(gcf,sprintf('EncountersTimes.png'),'-dpng');
        cropBackground(sprintf('EncountersTimes.png'));
        print(gcf,sprintf('EncountersTimes.eps'),'-depsc');
        
    
    mapEncounters=parseEncounters(mapEncounters,1);
        
    figure(106)
        imagesc(1-MAP)
        axis image;
        axis off
        colormap gray;
        hold on;
        for a1=1:size(mapEncounters,1)
            t=(mapEncounters(a1,1)-1):(mapEncounters(a1,1)+1);
            oldrob=mapEncounters(a1,2);
            newrob=mapEncounters(a1,3);
            relpos=mapEncounters(a1,4:5);
            
            plot(data(newrob).pose(2,t),data(newrob).pose(1,t),'Color',colours(a1,:))
            plot(data(oldrob).pose(2,t(2))+relpos(2),data(oldrob).pose(1,t(2))+relpos(1),'o','Color',colours(a1,:))
            plot([data(oldrob).pose(2,t(2)) data(oldrob).pose(2,t(2))+relpos(2)],[data(oldrob).pose(1,t(2)) data(oldrob).pose(1,t(2))+relpos(1)],'-s','Color',colours(a1,:))
        end
        hold off;
        
        
        
    return;    
    figure(104)
    
        for a1=1:size(data(1).pose,2)
            colours=lines(numel(data));
                imagesc(1-MAP')
                axis image;
                axis off
                colormap gray;
                hold on;
                for a2=1:numel(data)
                    plot(data(a2).pose(1,1:a1),data(a2).pose(2,1:a1),'Color',colours(a2,:))
                    hold on;
                end
            
            inds=a1==mapEncounters(:,1);
            evect=1:size(mapEncounters,1);
            evect=evect(inds);
            if (sum(inds))
                for a2=1:sum(inds)
                    x=[data(mapEncounters(evect(a2),2)).pose(1,mapEncounters(evect(a2),1)) data(mapEncounters(evect(a2),3)).pose(1,mapEncounters(evect(a2),1))];
                    y=[data(mapEncounters(evect(a2),2)).pose(2,mapEncounters(evect(a2),1)) data(mapEncounters(evect(a2),3)).pose(2,mapEncounters(evect(a2),1))];
                    plot(x,y,'--','Color',[0.5 0.5 0.5],'LineWidth',2)
                end
            end
            hold off;
            drawnow;
            print(gcf,sprintf('Connectivity/Connectivity-%03d.png',a1),'-dpng');
        end
    
        
end