function plotConnectivity(MAP,mapEncounters,data)

    fontsize=14;
    fontname='Times';


    if (nargin==0)
        load('CustomData')
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
    
    figure(103)
        colours=lines(numel(data));
        imagesc(1-MAP)
        axis image;
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
end