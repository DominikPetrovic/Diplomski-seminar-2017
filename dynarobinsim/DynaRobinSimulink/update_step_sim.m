function update_step(x_m, delta_x_m, dt,  P_m, encoders, footZ)
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


%%
H=[-C_m zeros(3) skew_matrix(C_m*(p1_m-r_m))  C_m zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p2_m-r_m))  zeros(3) C_m zeros(3) zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p3_m-r_m))  zeros(3) zeros(3) C_m zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p4_m-r_m))  zeros(3) zeros(3) zeros(3) C_m zeros(3) zeros(3)];

%%

if(footZ(1)>-0.254)
    ground_contactFL=0;
    Ra1=99999;
else
    Ra1=ones(4)*1; Rs1=ones(3)*1; 
    ground_contactFL=1;
end
if(footZ(2)>-0.254)
    ground_contactFR=0;
    Ra2=99999;
else
    Ra2=ones(4)*1; Rs2=ones(3)*1; 
    ground_contactFR=1;
end
if(footZ(3)>-0.254)
    ground_contactBL=0;
    Ra3=99999;
else
    Ra3=ones(4)*1; Rs3=ones(3)*1; 
    ground_contactBL=1;
end    
if(footZ(4)>-0.254)
    ground_contactBR=0;
    Ra4=99999;
else
    Ra4=ones(4)*1; Rs4=ones(3)*1; 
    ground_contactBR=1;
end


%Ra1=rand(4)*1; Rs1=rand(3)*1;
q01=encoders(1:3)';
%Rlkin1=jacobian_dir_kin(q01,'FL');       
Rlkin1=jacobian_dir_kin2(q01);  
%covariance matrix for ni
R1=double(Rs1+Rlkin1*Ra1*Rlkin1');

%Ra2=rand(4)*1; Rs2=rand(3)*1;
q02=encoders(4:6)';
%Rlkin2=jacobian_dir_kin(q02,'FR');   
Rlkin2=jacobian_dir_kin2(q02);  
%covariance matrix for ni
R2=double(Rs2+Rlkin2*Ra2*Rlkin2');

%Ra3=rand(4)*1; Rs3=rand(3)*1; 
q03=encoders(7:9)';
%Rlkin3=jacobian_dir_kin(q03,'BL');  
Rlkin3=jacobian_dir_kin2(q03);  
%covariance matrix for ni
R3=double(Rs3+Rlkin3*Ra3*Rlkin3');

%Ra4=rand(4)*1; Rs4=rand(3)*1;
q04=encoders(10:12)';
%Rlkin4=jacobian_dir_kin(q04,'BR'); 
Rlkin4=jacobian_dir_kin2(q04);  
%covariance matrix for ni
R4=double(Rs4+Rlkin4*Ra4*Rlkin4');

R=[ R1 zeros(3) zeros(3) zeros(3);
   zeros(3) R2 zeros(3) zeros(3);
   zeros(3) zeros(3) R3 zeros(3); 
   zeros(3) zeros(3) zeros(3) R4];

assignin('base','R',R);
assignin('base','H',H);
S=H*P_m*H'+R;
assignin('base','S',S);
K=P_m*H'*S^-1;
assignin('base','K',K);
delta_x=K*y;
P_p=(eye(27)-K*H)*P_m;

delta_fi_p=delta_x(7:9);
delta_q_p=quatmultiply(map_q(delta_fi_p)',q_m')';

x_state(1:6,1)=x_m(1:6)+delta_x(1:6);
x_state(7:10,1)=delta_q_p;
x_state(11:28,1)=x_m(11:28)+delta_x(10:27);

%% saving to matlab workspace

assignin('base','x_state',x_state);
assignin('base','delta_x',delta_x);
assignin('base','P_p', P_p);


end