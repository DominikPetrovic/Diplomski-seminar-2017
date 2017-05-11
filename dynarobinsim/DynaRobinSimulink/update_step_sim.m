function update_step(x_m, delta_x_m, dt,  P_m, encoders)
% innovation - difference between actual measurements and their predicted value

r_m=x_m(1:3);
v_m=x_m(4:6);
q_m=x_m(7:10); C_m=quat2rotm(q_m');
p1_m=x_m(11:13); p2_m=x_m(14:16); p3_m=x_m(17:19); p4_m=x_m(20:22);%p=1x12, pi=3x1 %i=1:4
bf_m=x_m(23:25);
bw_m=x_m(26:28);

delta_r_m=delta_x_m(1:3);
delta_v_m=delta_x_m(4:6);
delta_fi_m=delta_x_m(7:9); 
delta_q_m=map_q(delta_fi_m);
delta_p1_m=delta_x_m(10:12); delta_p2_m=delta_x_m(13:15); delta_p3_m=delta_x_m(16:18); delta_p4_m=delta_x_m(19:21);%p=1x12, pi=3x1 %i=1:4
delta_p=[delta_p1_m; delta_p2_m; delta_p3_m; delta_p4_m];
delta_bf_m=delta_x_m(22:24);
delta_bw_m=delta_x_m(25:27);


y=[C_m*delta_r_m+C_m*delta_p1_m+skew_matrix(C_m*(p1_m-r_m))*delta_fi_m;
   C_m*delta_r_m+C_m*delta_p2_m+skew_matrix(C_m*(p2_m-r_m))*delta_fi_m;
   C_m*delta_r_m+C_m*delta_p3_m+skew_matrix(C_m*(p3_m-r_m))*delta_fi_m;
   C_m*delta_r_m+C_m*delta_p4_m+skew_matrix(C_m*(p4_m-r_m))*delta_fi_m;];

%% Encoders measurements

% da li se tu ubacuju mjerenja enkodera i direktna kinematika????
%y=[s1-C_m*(p1_m);
%   s2-C_m*(p2_m);
%   s3-C_m*(p3_m);
%  s4-C_m*(p4_m);];


%%
H=[-C_m zeros(3) skew_matrix(C_m*(p1_m-r_m))  C_m zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p2_m-r_m))  zeros(3) C_m zeros(3) zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p3_m-r_m))  zeros(3) zeros(3) C_m zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p4_m-r_m))  zeros(3) zeros(3) zeros(3) C_m zeros(3) zeros(3)];

%%
Ra=ones(4); Rs=ones(3); q01=rand(1,4); %poslije promjeniti
q01=encoders(1:4)';
Rlkin1=jacobian_dir_kin(q01,'FR');        
%covariance matrix for ni
R1=double(Rs+Rlkin1*Ra*Rlkin1'); 

Ra=ones(4); Rs=ones(3); q02=rand(1,4); %poslije promjeniti
q02=encoders(5:8)';
Rlkin2=jacobian_dir_kin(q02,'FL');        
%covariance matrix for ni
R2=double(Rs+Rlkin2*Ra*Rlkin2');

Ra=ones(4); Rs=ones(3); q03=rand(1,4); %poslije promjeniti
q03=encoders(9:12)';
Rlkin3=jacobian_dir_kin(q03,'BR');        
%covariance matrix for ni
R3=double(Rs+Rlkin3*Ra*Rlkin3');

Ra=ones(4); Rs=ones(3); q04=rand(1,4); %poslije promjeniti
q04=encoders(13:16)';
Rlkin4=jacobian_dir_kin(q04,'BL');        
%covariance matrix for ni
R4=double(Rs+Rlkin4*Ra*Rlkin4');

R=[ R1 zeros(3) zeros(3) zeros(3);
   zeros(3) R2 zeros(3) zeros(3);
   zeros(3) zeros(3) R3 zeros(3); 
   zeros(3) zeros(3) zeros(3) R4];


S=H*P_m*H'+R;
K=P_m*H'*S^-1;
delta_x=K*y;
P_p=(eye(27)-K*H)*P_m;

delta_fi_p=delta_x(7:9);
delta_q_p=quatmultiply(map_q(delta_fi_p)',q_m')';

x(1:6,1)=x_m(1:6)+dt*delta_x(1:6);
x(7:10,1)=x_m(7:10)+dt*delta_q_p;
x(11:28,1)=x_m(11:28)+dt*delta_x(10:27);

%% saving to matlab workspace

assignin('base','x',x);
assignin('base','delta_x',delta_x);
assignin('base','P_p', P_p);


end