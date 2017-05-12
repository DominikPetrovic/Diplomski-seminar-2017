function prediction_step(x, delta_x, dt, f_IMU, w_IMU, P_p)

%suffix p stands for plus
%suffix m stands for minus, ( minus is a priori state estimation in step
%k+1) while p is in step k

r_p=x(1:3);
v_p=x(4:6);
q_p=x(7:10);
p1_p=x(11:13); p2_p=x(14:16); p3_p=x(17:19); p4_p=x(20:22);%p=1x12, pi=3x1 %i=1:4
bf_p=x(23:25);
bw_p=x(26:28);

f_tilda=f_IMU; % measured f
w_tilda=w_IMU; % measured w

delta_r_p=delta_x(1:3);
delta_v_p=delta_x(4:6);
delta_fi_p=delta_x(7:9);
delta_q_p=map_q(delta_fi_p);
delta_p1_p=delta_x(10:12); delta_p2_p=delta_x(13:15); delta_p3_p=delta_x(16:18); delta_p4_p=delta_x(19:21);%p=1x12, pi=3x1 %i=1:4
delta_p=[delta_p1_p; delta_p2_p; delta_p3_p; delta_p4_p];
delta_bf_p=delta_x(22:24);
delta_bw_p=delta_x(25:27);

C_p=quat2rotm(q_p'); %rotation matrix
f_hat=f_tilda-bf_p; % bias corrected IMU measurements
w_hat=w_tilda-bw_p; % bias corrected IMU measurements
g=[0 0 0]';
%% a priori state estimate equations 
r_m=r_p+dt*v_p+dt^2/2*(C_p'*f_hat+g);
v_m=v_p+dt*(C_p'*f_hat+g);
q_m=quatmultiply(map_q(dt*w_hat)',q_p')';
p1_m=p1_p; p2_m=p2_p; p3_m=p3_p; p4_m=p4_p;
bf_m=bf_p;
bw_m=bw_p;

p_m=[p1_m; p2_m; p3_m; p4_m];

x_m=[r_m; v_m; q_m; p_m; bf_m; bw_m];

%% set of linear differential equations describing the error dynamics
ww=wgn(3,1,0); wbw=wgn(3,1,0); % ??????
wf=wgn(3,1,0); wbf=wgn(3,1,0); % ??????

%% potrebno promjeniti
ww=zeros(3,1); wbw=zeros(3,1); 
wf=zeros(3,1); wbf=zeros(3,1); 

%%
d_delta_r=delta_v_p;
d_delta_v=-C_p'*skew_matrix(f_tilda)*delta_fi_p-C_p'*delta_bf_p-C_p'*wf;
d_delta_fi=-skew_matrix(w_tilda)*delta_fi_p-delta_bw_p-ww;

Qp_1=[rand(1,1) 0 0; 0 rand(1,1) 0; 0 0 rand(1,1)]; wp_1=mvnrnd([0 0 0],Qp_1)'; % ??? 
Qp_2=[rand(1,1) 0 0; 0 rand(1,1) 0; 0 0 rand(1,1)]; wp_2=mvnrnd([0 0 0],Qp_2)'; % ???
Qp_3=[rand(1,1) 0 0; 0 rand(1,1) 0; 0 0 rand(1,1)]; wp_3=mvnrnd([0 0 0],Qp_3)'; % ???
Qp_4=[rand(1,1) 0 0; 0 rand(1,1) 0; 0 0 rand(1,1)]; wp_4=mvnrnd([0 0 0],Qp_4)'; % ???

%% potrebno promijeniti
wp_1=[0 0 0]';
wp_2=[0 0 0]';
wp_3=[0 0 0]';
wp_4=[0 0 0]';

%%
d_delta_p1=C_p'*wp_1;
d_delta_p2=C_p'*wp_2;  d_delta_p3=C_p'*wp_3; d_delta_p4=C_p'*wp_4;
d_delta_p=[d_delta_p1; d_delta_p2; d_delta_p3; d_delta_p4];
d_delta_bf=wbf;
d_delta_bw=wbw;

d_delta_x=[d_delta_r; d_delta_v; d_delta_fi; d_delta_p; d_delta_bf; d_delta_bw];
 
delta_x_m=delta_x+dt*d_delta_x;
    
%% %% discrete linearized error dynamics matrix Fk

%first column
F(1:3,1:3)=ones(3);
F(4:6,1:3)=zeros(3);
F(7:9,1:3)=zeros(3,3);
F(10:21,1:3)=zeros(12,3);
F(22:24,1:3)=zeros(3);
F(25:27,1:3)=zeros(3);

%second column
F(1:3,4:6)=dt*ones(3);
F(4:6,4:6)=ones(3);
F(7:9,4:6)=zeros(3,3);
F(10:21,4:6)=zeros(12,3);
F(22:24,4:6)=zeros(3);
F(25:27,4:6)=zeros(3);

%third column
F(1:3,7:9)=-dt^2/2*C_p'*skew_matrix(f_tilda);
F(4:6,7:9)=-dt*C_p'*skew_matrix(f_tilda);
F(7:9,7:9)=map_gama(0,dt,w_tilda);
F(10:21,7:9)=zeros(12,3);
F(22:24,7:9)=zeros(3,3);
F(25:27,7:9)=zeros(3,3);

%fourth column
F(1:3,10:21)=zeros(3,12);
F(4:6,10:21)=zeros(3,12);
F(7:9,10:21)=zeros(3,12);
F(10:21,10:21)=ones(12,12);
F(22:24,10:21)=zeros(3,12);
F(25:27,10:21)=zeros(3,12);

%fifth column
F(1:3,22:24)=-dt^2/2*C_p';
F(4:6,22:24)=-dt*C_p';
F(7:9,22:24)=zeros(3,3);
F(10:21,22:24)=zeros(12,3);
F(22:24,22:24)=ones(3);
F(25:27,22:24)=zeros(3);

%sixth column
F(1:3,25:27)=zeros(3);
F(4:6,25:27)=zeros(3);
F(7:9,25:27)=-map_gama(1,dt,w_tilda);
F(10:21,25:27)=zeros(12,3);
F(22:24,25:27)=zeros(3);
F(25:27,25:27)=ones(3);
      
%% discrete process noise covariance matrix Qk

%covariance parameters
Qf=rand(3,3);
Qbf=rand(3,3);
Qw=rand(3,3);
Qbw=rand(3,3);
Qp=rand(3,3);

Q=zeros(27);

%first column
Q(1:3,1:3)=dt^3/3*Qf+dt^5/20*Qbf;
Q(4:6,1:3)=dt^2/2*Qf+dt^4/8*Qbf;
Q(22:24,1:3)=dt^3/6*Qbf*C_p;

%second column
Q(1:3,4:6)=dt^2/2*Qf+dt^4/8*Qbf;
Q(4:6,4:6)=dt*Qf+dt^3/3*Qbf;
Q(22:24,4:6)=-dt^2/2*Qbf*C_p;

%third column
Q(7:9,7:9)=dt*Qw+(map_gama(3,dt,w_tilda)+map_gama(3,dt,w_tilda))*Qbw;
Q(25:27,7:9)=-Qbw*map_gama(2,dt,w_tilda);

%fourth column
Q(10:12,10:12)=dt*C_p'*Qp*C_p;

%fith column
Q(1:3,22:24)=-dt^3/6*C_p'*Qbf;
Q(4:6,22:24)=-dt^2/2*C_p'*Qbf;
Q(22:24,22:24)=dt*Qbf;

%sixth column
Q(7:9,25:27)=-map_gama(2,dt,w_tilda)*Qbw;
Q(25:27,25:27)=dt*Qbw;


%% a priori estimate of the covariance matrix at the timestep k + 1
P_m=F*P_p*F'+Q;

%% saving to matlab workspace
assignin('base','F',F);
assignin('base','Q',Q);
assignin('base','P_m',P_m);
assignin('base','x_m',x_m);
assignin('base','delta_x_m',delta_x_m);



end
      
      
      
      
      
      
      
      
      
      
      
      
      
      
