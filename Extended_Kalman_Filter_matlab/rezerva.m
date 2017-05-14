clear all
clc 

dt=1;
syms q1 q2 q3 q4

%parametri robota
a1=0; a2=0.1002; a3=0.20425; a4=0.01908;
d1=0; d2=0.053;  d3=0; d4=0;
q4=29.04*pi/180;
save params


%r=[0.5 0.2 0.3]';
%v=[0.1 0.05 0.03]';
%qvat=[0.1 0.2 0.1 0.2]';
%p=[1 2 3 4; 
%   2 2 2 2;
%   3 3 3 3];
%p1=p(1:3,1); p2=p(1:3,2); p3=p(1:3,3); p4=p(1:3,4);
%p=[p1; p2; p3; p4];
%bf=[0.02; 0.02; 0.02];
%bw=[0.05; 0.05; 0.05];
%q0=[0.23 pi/3 1.2 0];

%x_state=[r; v; qvat; p; bf; bw];

% xP=predict(x_state,dt) % ili ovo?
% 
% rNew=xP(1:3)';
% vNew=xP(4:6)';
% qNew=xP(7:10)';
% pNew=xP(11:22);
% bfNew=xP(23);
% bwNew=xP(24);
% p1New=pNew(1:3); p2New=pNew(4:6); p3New=pNew(7:9); p4New=pNew(10:12);
% 
% s= measurement(qvat,p,r',q0)


%w=dir_kin(q0,'FR');

%q=inv_kin(w,'FR');

% Jacobian 3x4 ????

%%pocetno stanje
x_state=rand(28,1);
delta_x=rand(27,1);
P_p=rand(27);

for i=1:5
measurements=rand(6,1);
[F Q P_m x_m delta_x_m]= prediction_step(x_state, delta_x, dt, measurements, P_p);

encoders=rand(1,16);
[x_state, delta_x_p, P_p]= update_step(x_m, delta_x_m, dt, P_m, encoders);

delta_x=delta_x_p;
end


