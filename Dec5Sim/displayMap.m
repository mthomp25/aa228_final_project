function displayMap(p,env)

for i = 1:length(p)
    [rr,cr] = ind2sub([env.rows,env.rows],p);
end

[rh,ch] = ind2sub([env.rows,env.rows],env.home);

nS = size(env.samples,2);

clf

figure(1); hold on

subplot(3,2,[1 2 3 4]); hold on
    grid on
    plot(cr-.5, -(rr-.5), 'r:')
    plot(ch-0.5,-(rh-0.5),'ko') %home
    for s = 1:nS
        [rs,cs] = ind2sub([env.rows,env.rows],env.samples(1,s));
        plot(cs-0.5,-(rs-0.5),'g*') %sample
    end
    plot(cr(end)-0.5,-(rr(end)-0.5),'r+') %rover
    axis([0 env.rows -env.rows 0])
    axis square
hold off
subplot(3,2,5); hold on
    imagesc(flipud(env.topo))
    plot(cr, -(rr)+env.rows+1, 'r:')
    title('Topography')
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    axis([0.5 env.rows+0.5 0.5 env.rows+0.5])
    axis square
hold off
subplot(3,2,6); hold on
    imagesc(flipud(env.zones))
    plot(cr, -(rr)+env.rows+1, 'r:')
    title('Terrain Zones')
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    axis([0.5 env.rows+0.5 0.5 env.rows+0.5])
    axis square
hold off
hold off
