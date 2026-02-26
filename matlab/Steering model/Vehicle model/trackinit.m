%
% Base track viewer - Driver model tests 2017
%

coords = [844,1468; 1420,2250; 1445,4330; 2200,7150; 3000,9900; 4900,12720; 5700,14500; 3940,15850; 1850,18550; -2060,19100; -2130,18285; -4725,16982; -5800,16800; -7560,13600; -9090,11450; -8430,9810; -7445,8545; -5100,5900; -7200,4400; -7700,2800; -9390,650; -7550,-2890];
places = {'East Leake start','East Leake','Costock','Wysall','Widmerpool','Pre Kin','Kinoulton','Hickling','Long Clawson','Ab Kettleby','Start SINGLE','End SINGLE','Asfordby','Hoby','Thrussington','A46x','Seagrave','Walton','Barrow','Quorn','Woodhouse','Nanpantan'};


load base_080617     % Fix to pre-Kin (seemingly small overtake removed, not in driver data)
Xt = Xb;
Yt = Yb;

roadwidth = 6.0;    % Can make a difference, but not as big as bloody mindedness from driver !
leftlim = -roadwidth/4;
centrelim = roadwidth/4;
rightlim = 3*roadwidth/4;

starts = [Xt,Yt]';
nel = length(Yt)-1;
% Pre-compute and store all useful vector information, in straight line and
% circle data arrays.
tss = [];
tsdir = [];
tslen = [];
tsnor = [];
rss = [];
css = [];
lss = [];
for i=1:nel
    tss = [tss, starts(:,i)];
    vec = starts(:,i+1) - starts(:,i);
    vecmag = sqrt(vec'*vec);
    vecdir = vec/sqrt(vec'*vec);
    tslen = [tslen, vecmag];
    tsdir = [tsdir, vecdir];
    rightperp = [-vecdir(2), vecdir(1)]';      % right facing unit perpendicular to heading direction
    tsnor = [tsnor, rightperp];
    rss = [rss, starts(:,i)+rightlim*rightperp];
    css = [css, starts(:,i)+centrelim*rightperp];
    lss = [lss, starts(:,i)+leftlim*rightperp];
end


% Show summary of track being used :
figure(1),clf
h = plot(Yb,Xb,'r',coords(:,2),coords(:,1),'bo');
figure(1), hold on
plot(lss(2,:),lss(1,:),'k-.',rss(2,:),rss(1,:),'k-.',css(2,:),css(1,:),'k--');
axis equal
for i=1:length(places)
    text(coords(i,2),coords(i,1),places{i},'fontsize',8);
end

% Example section from driver (between Hoby and Thrussington)
load driver1
h=plot(Y15,X15,'b');
set(h,'linewidth',2)
hold off
















