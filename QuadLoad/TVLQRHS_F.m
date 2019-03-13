close all, clear all,clc

p = QuadLoadPlant2D2I();
x0 =  [0; 0; 0; 0; 0; 0;   0; 0; 0; 0; 0; 0];

mod_seq=[1;2;1];
load('YTPCdone.mat')
j=length(ytraj.traj)+1;

Q = 100*diag([ones(6,1); 0.1*ones(6,1)]);
R = 0.1*eye(2);
Qf = Q;
Qff = Qf;
ind_dif=length(ytraj.traj)-length(mod_seq);

for i=(length(mod_seq):-1:1)
curr_mode=mod_seq(i);
if i>1
    last_mode=mod_seq(i-1);
end

j=j-1;
load('YTPCdone.mat')
load('UTPCdone.mat')

xtraj = ytraj;
if j>1
    xtraj_last = xtraj.traj{j-1};
    utraj_lastval = utraj.eval(xtraj_last.tspan(2));
end

xtraj = xtraj.traj{j};
xtraj = xtraj(2:13);



ts = xtraj.getBreaks();

utraj = PPTrajectory(spline(ts,utraj.eval(ts))); 

xtraj = xtraj.setOutputFrame(p.modes{curr_mode}.getStateFrame);
utraj = utraj.setOutputFrame(p.modes{curr_mode}.getInputFrame);



%%
% tvlqr 
options = struct();
options.sqrtmethod = false;
[tv,V] = tvlqr(p.modes{curr_mode},xtraj,utraj,Q,R,Qf,options);
QfV = Qf;

%%
if i>1
ti=find(p.target_mode{last_mode}==curr_mode);
TransDyna=p.transition{last_mode}{ti};
[xp,mode,status,dxp] = TransDyna(p,last_mode,xtraj_last.tspan(2),xtraj_last.eval(xtraj_last.tspan(2)),utraj_lastval);
Ad=dxp(:,3:14);

S_t_plus = V.S.eval(ts(1));%CHECK
S_t_minus = Ad'*S_t_plus*Ad;
Qf = S_t_minus;
end

tv.y0=tv.y0+(utraj-tv.D*xtraj);
Tv{j}=tv;
Xtraj{j}=xtraj;
Utraj{j}=utraj;
Ts{j}=ts;

end

%%
Tv2={Tv{(1:length(mod_seq))+ind_dif}};
cellTD{1}=[[0,0];(Tv2{1}.D)']';
for i=1:length(Tv2)
    cellTD{i}=[[0,0];(Tv2{i}.D)']';
    cellTD{i}=cellTD{i}.setOutputFrame(cellTD{1}.getOutputFrame);
    cellTy0{i}=Tv2{i}.y0;
    cellTy0{i}=cellTy0{i}.setOutputFrame(Tv2{1}.y0.getOutputFrame);
end
TvHD=HybridTrajectory(cellTD);
TvHy0=HybridTrajectory(cellTy0);

% tv.D=;
% tv.y0=;
tv = AffineSystem([],[],[],[],[],[],[],TvHD,TvHy0);
%%
p=QuadLoadPlant2D2I();
tv=tv.setOutputFrame(p.getInputFrame);
tv=tv.setInputFrame(p.getOutputFrame);
%%

sys_cl = p.feedback(tv);

%%
x0_H =  [1; 0; 0; 0; 0; 0; 0;   0; 0; 0; 0; 0; 0];
xf0_H = x0_H + [0; 0.05*(2*rand(12,1)-1)];
xf0_H(2) = -1;
xf0_H(3) = -1;
% xf0_H(4) = 0.2;
% xf = xf0;

traj_testf = sys_cl.simulate([0 7.1496],xf0_H); 
%plottingsim_wdw_TVLQR3
%
plottingsim_wdw_TVLQR