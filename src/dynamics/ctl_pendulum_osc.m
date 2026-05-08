%% 
clear
clc
close all

s = tf('s');

Gr = 2/s / (1/(2*pi*2)*s+1);
Kr = tf(pid(0.1,0.1,0)) * (1/(2*pi*2)*s+1)/(1/(2*pi*20)*s+1) * 100;

Gr_cl = feedback(Gr*Kr,1);

Ge = Gr_cl / s;
Ke = 6;

filt2  = d2c(tf(0.000241359*[1 2 1],[1 -1.95557824 0.9565436765], 0.0025));
Ge_cl = feedback(Ge*Ke,1) * filt2;

Gv = Ge_cl / s;
Kv = tf(pid(.5,0.02,0)) * (1 + 0.6*s);

Gv_cl = feedback(Gv*Kv,1);

figure;
opt = bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
% opt.Xlim = [1e-4 10];
% bode(Gr_cl,Ge_cl,Gv_cl,opt);
% 
% figure;
r2a = Kv*Ge_cl/(1+Kv*Gv);
% bode(r2a, opt);

%% 
l = 10;
g = 9.81;

A = [0 1;-g/l 0];
B = [0;1/l];
C = eye(2);
D = 0;

pend_ss = ss(A,B,C,D);
pend_tf=-tf(pend_ss);
pend_w = pend_tf(2);
pend_eul = pend_tf(1);

r2pend = r2a * pend_tf;

% figure;
% bode(r2pend,opt);
% title('r-pend')

figure;
step(pend_tf);grid
title('pend step')

%% plant
%
%                                                 +-------------------------|
%               a       eul     eul         a1    | a                       |
%  r--->o--->Kv--->gain----->P------>1/gain--+--->o--->1/s----->y           |
%       |                                    |              |               |
%       -------------------------------------+---------------               |
%                                            |                              |a2
%                                            +--->pend---->w,eul----gain1---+
%
%   gain1: w^2*l*eul*m/M

gain = 1/g/180*pi;
P = Kv*gain * Ge_cl / gain;
P.InputName = 'e';
P.OutputName = 'a1';

pend = -pend_eul*g * 1.5; 
pend.InputName = 'a1';
pend.OutputName = 'a2';

% lift = pend_eul *g;
% lift.InputName = 'weul';
% lift.OutputName = 'a2';

Inte = 1/s;
Inte.InputName = 'a';
Inte.OutputName = 'v';

S2 = sumblk('e = r-v');
S1 = sumblk('a = a1 + a2');
T = connect(P,pend,Inte,S1,S2,{'r'},{'v','a1','a2'})
zpk(T)
zpk(Ge_cl*gain*pend_eul*g)
zpk(pend_tf)

close all
figure;
bode(T,opt)
title('T bode')

t = 0 : 0.01 : 10;
u = sin(2*pi*.0*t);
x0 = zeros(11,1);
x0(10) = -30/180*pi;
y = lsim(T, u, t, x0);
figure;
subplot(3,1,1);plot(t,y(:,1));grid;hold on;
plot(t,u,'r');
title('r-v')
subplot(3,1,2);plot(t,y(:,2));grid
title('r-a1')
subplot(3,1,3);plot(t,y(:,3));grid
title('r-a2')

% figure;
% filter = 1/(1/(2*pi*0.01)*s+1);
% step(T*filter);grid
% title('pend step')

% [y,t] = step(T);grid
% subplot(1,2,1);plot(t,y(:,3));
% subplot(1,2,2);plot(t,y(:,4));


% [y,t] = step(T);
% pend_a = y(:,3);
% figure;
% plot(t,pend_a);grid
% title('pend a')