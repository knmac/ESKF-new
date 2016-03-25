close all; clear all; clc;

ids = {'exp5-stand', 'exp6-1bending', 'exp7-3bending', 'exp8-leftright', ...
    'exp9-backforth', 'exp10-backforth2', 'exp11-kitchen', 'exp12-kitchen2', ...
    'exp13-corridor', 'exp14-uturn', 'exp15-stair', 'exp16-room1turn', ...
    'exp17-longroomp1', 'exp18-longroomp2', 'exp19-longroomp3', 'exp20-longroomall', ...
    'exp21-longroomall2', 'exp22-room3turns', 'exp23-room6turns', 'exp24-corridor2', ...
    'exp25-corridor3', 'exp26-stair2', 'exp27-kitchen3', 'exp28-blankhallway', ...
    'exp29-longhallway'};

% ids = {'exp5-stand'};

posfix = {'p','v','q','ab','wb','g'};

for k = 1:numel(ids)
    close all;
    id = ids{k};
    fprintf('%s\n',id');
    load([id,'.mat']);
    
    Visualizer;
    
    for l = 1:6
        h = figure(l);
        set(h, 'Units', 'inches');
        pos = get(h,'Position');
        set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
        set(gca,'FontSize',20);
        set(get(gca,'xlabel'),'fontsize',15)
        set(get(gca,'ylabel'),'fontsize',15)
        set(get(gca,'zlabel'),'fontsize',15)
        
        filename = ['fig/',id,'-',posfix{l},'.pdf'];        
        print(h,filename,'-dpdf','-r0');
    end
end