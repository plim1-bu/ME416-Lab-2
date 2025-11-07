function dubins_xy_full()
% DUBINS_XY_FULL
% Prompts for start/goal pose and radius, computes & plots 6 Dubins families
% LSL, RSR, LSR, RSL, RLR, LRL. Prints per-segment details + optimal path.
%
% Units: meters and radians (headings input in degrees).
% No toolboxes required.

clc; close all; clearvars;

fprintf('--- Dubins Paths (CSC + CCC) ---\n');
xs = input('Start X (m): ');
ys = input('Start Y (m): ');
ths_deg = input('Start heading (deg, 0=+x, CCW+): ');
xg = input('Goal  X (m): ');
yg = input('Goal  Y (m): ');
thg_deg = input('Goal  heading (deg): ');
R  = input('Turning radius R (m): ');

ths = deg2rad(ths_deg);
thg = deg2rad(thg_deg);

% Transform goal into start-aligned, unit-radius frame
[x, y, phi] = transformToStartFrame(xs, ys, ths, xg, yg, thg, R);

% Aux vars
d     = hypot(x, y);
theta = atan2(y, x);
alpha = wrap2pi(-theta);
beta  = wrap2pi(phi - theta);

% Candidate families
cands = { ...
   'LSL', +1,  0, +1, @solve_LSL; ...
   'RSR', -1,  0, -1, @solve_RSR; ...
   'LSR', +1,  0, -1, @solve_LSR; ...
   'RSL', -1,  0, +1, @solve_RSL; ...
   'RLR', -1, +1, -1, @solve_RLR; ...
   'LRL', +1, -1, +1, @solve_LRL; ...
};

results = struct('name',{},'seq',{},'t',{},'p',{},'q',{},'L',{},'feasible',{});
for i = 1:size(cands,1)
    name = cands{i,1};
    seq  = [cands{i,2}, cands{i,3}, cands{i,4}];
    solver = cands{i,5};
    [t,p,q,ok] = solver(alpha, beta, d);
    L = (t + p + q) * R;
    results(i) = struct('name',name,'seq',seq,'t',t,'p',p,'q',q,'L',L,'feasible',ok);
end

% Report
fprintf('\nPath breakdowns (angles in rad, lengths in m):\n');
bestIdx = []; bestL = inf;
for i = 1:numel(results)
    r = results(i);
    if r.feasible
        % segment-by-segment
        if r.seq(2)==0
            % CSC
            seg1 = r.t*R; seg2 = r.p*R; seg3 = r.q*R;
            fprintf('  %-3s : total %.3f m\n', r.name, r.L);
            fprintf('        Turn1=%.3f m, Straight=%.3f m, Turn2=%.3f m\n', seg1, seg2, seg3);
        else
            % CCC
            seg1 = r.t*R; seg2 = r.p*R; seg3 = r.q*R;
            fprintf('  %-3s : total %.3f m\n', r.name, r.L);
            fprintf('        Turn1=%.3f m, Turn2=%.3f m, Turn3=%.3f m\n', seg1, seg2, seg3);
        end
        if r.L < bestL
            bestL = r.L;
            bestIdx = i;
        end
    else
        fprintf('  %-3s : infeasible\n', r.name);
    end
end

if isempty(bestIdx)
    error('No feasible Dubins solution found.');
else
    fprintf('\nOptimal path: %s (%.3f m)\n', results(bestIdx).name, results(bestIdx).L);
end

% Plot
figure('Color','w'); hold on; axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)'); title('Dubins Paths (CSC + CCC)');
drawPose(xs, ys, ths, 'g', 0.7*R, 'Start');
drawPose(xg, yg, thg, 'm', 0.7*R, 'Goal');

col = struct('LSL',[0 0.4470 0.7410], ...
             'RSR',[0.8500 0.3250 0.0980], ...
             'LSR',[0.9290 0.6940 0.1250], ...
             'RSL',[0.4940 0.1840 0.5560], ...
             'RLR',[0.4660 0.6740 0.1880], ...
             'LRL',[0.3010 0.7450 0.9330]);

legendEntries = {};
for i = 1:numel(results)
    if ~results(i).feasible, continue; end
    c = col.(results(i).name);
    XY = sampleDubinsWorld(xs, ys, ths, R, results(i).seq, results(i).t, results(i).p, results(i).q);
    plot(XY(:,1), XY(:,2), 'LineWidth', 2, 'Color', c);
    legendEntries{end+1} = sprintf('%s (%.2f m)', results(i).name, results(i).L); %#ok<AGROW>
end
legend(legendEntries, 'Location','bestoutside');
hold off;

end

% ---------- Solvers (CSC + CCC) ----------
function [t,p,q,ok] = solve_LSL(alpha,beta,d)
p2 = 2 + d^2 - 2*cos(alpha-beta) + 2*d*(sin(alpha)-sin(beta));
if p2<0, ok=false; t=NaN;p=NaN;q=NaN; return; end
p = sqrt(p2);
ang = atan2((cos(beta)-cos(alpha)), (d+sin(alpha)-sin(beta)));
t = wrap2pi(-alpha+ang);
q = wrap2pi(beta-ang);
ok=true;
end

function [t,p,q,ok] = solve_RSR(alpha,beta,d)
p2 = 2 + d^2 - 2*cos(alpha-beta) + 2*d*(-sin(alpha)+sin(beta));
if p2<0, ok=false; t=NaN;p=NaN;q=NaN; return; end
p = sqrt(p2);
ang = atan2((cos(alpha)-cos(beta)), (d-sin(alpha)+sin(beta)));
t = wrap2pi(alpha-ang);
q = wrap2pi(-beta+ang);
ok=true;
end

function [t,p,q,ok] = solve_LSR(alpha,beta,d)
p2 = -2 + d^2 + 2*cos(alpha-beta) + 2*d*(sin(alpha)+sin(beta));
if p2<0, ok=false; t=NaN;p=NaN;q=NaN; return; end
p = sqrt(p2);
ang = atan2((-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)));
t = wrap2pi(-alpha+ang-atan2(2,p));
q = wrap2pi(-beta+ang-atan2(2,p));
ok=true;
end

function [t,p,q,ok] = solve_RSL(alpha,beta,d)
p2 = -2 + d^2 + 2*cos(alpha-beta) - 2*d*(sin(alpha)+sin(beta));
if p2<0, ok=false; t=NaN;p=NaN;q=NaN; return; end
p = sqrt(p2);
ang = atan2((cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta)));
t = wrap2pi(alpha-ang+atan2(2,p));
q = wrap2pi(beta-ang+atan2(2,p));
ok=true;
end

function [t,p,q,ok] = solve_RLR(alpha,beta,d)
tmp = (6 - d^2 + 2*cos(alpha-beta) + 2*d*(sin(alpha)-sin(beta))) / 8;
if abs(tmp)>1, ok=false; t=NaN;p=NaN;q=NaN; return; end
p = wrap2pi(2*pi - acos(tmp));
ang = atan2((cos(alpha)-cos(beta)), (d-sin(alpha)+sin(beta)));
t = wrap2pi(alpha - ang + p/2);
q = wrap2pi(alpha-beta - t + p);
ok=true;
end

function [t,p,q,ok] = solve_LRL(alpha,beta,d)
tmp = (6 - d^2 + 2*cos(alpha-beta) + 2*d*(-sin(alpha)+sin(beta))) / 8;
if abs(tmp)>1, ok=false; t=NaN;p=NaN;q=NaN; return; end
p = wrap2pi(2*pi - acos(tmp));
ang = atan2((cos(beta)-cos(alpha)), (d+sin(alpha)-sin(beta)));
t = wrap2pi(-alpha + ang + p/2);
q = wrap2pi(beta-alpha - t + p);
ok=true;
end

% ---------- Helpers ----------
function a = wrap2pi(a)
a = mod(a,2*pi);
end

function [x,y,phi] = transformToStartFrame(xs,ys,ths,xg,yg,thg,R)
dx = (xg-xs)/R; dy = (yg-ys)/R;
c=cos(ths); s=sin(ths);
x =  c*dx + s*dy;
y = -s*dx + c*dy;
phi = wrap2pi(thg-ths);
end

function drawPose(x,y,th,col,arm,labeltxt)
plot(x,y,'ko','MarkerFaceColor',col,'MarkerSize',6);
plot([x x+arm*cos(th)],[y y+arm*sin(th)],'-','Color',col,'LineWidth',2);
text(x,y,['  ' labeltxt],'Color',col,'FontWeight','bold');
end

% ---------- Path sampling ----------
function XY = sampleDubinsWorld(xs,ys,ths,R,seq,t,p,q)
res=0.02*R; XY=[xs,ys]; pose=[xs;ys;ths];
pose=stepTurn(pose,seq(1),t*R,R,res); XY=[XY;pose(1:2)'];
if seq(2)==0
    pose=stepStraight(pose,p*R,res);
else
    pose=stepTurn(pose,seq(2),p*R,R,res);
end
XY=[XY;pose(1:2)'];
pose=stepTurn(pose,seq(3),q*R,R,res); XY=[XY;pose(1:2)'];
end

function pose=stepTurn(pose,sign,arcLen,R,res)
if sign==0||arcLen<=0, return; end
dth=(res/R)*sign; n=max(1,ceil(abs(arcLen)/res));
x=pose(1); y=pose(2); th=pose(3);
for k=1:n
    thn=th+dth;
    x=x+R*(sin(thn)-sin(th));
    y=y-R*(cos(thn)-cos(th));
    th=thn;
end
pose=[x;y;th];
end

function pose=stepStraight(pose,len,res)
if len<=0, return; end
n=max(1,ceil(len/res)); step=len/n;
x=pose(1); y=pose(2); th=pose(3);
for k=1:n
    x=x+step*cos(th); y=y+step*sin(th);
end
pose=[x;y;th];
end
