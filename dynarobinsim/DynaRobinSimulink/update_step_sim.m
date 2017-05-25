function update_step(x_m, delta_x_m, dt,  P_m, encoders, footZ, S, globalPos, angles, P, rotation)
% innovation - difference between actual measurements and their predicted value

r_m=x_m(1:3);
%v_m=x_m(4:6);
q_m=x_m(7:10); C_m=quat2rotm(q_m'); C_m=flip(C_m,1); C_m=flip(C_m,2); C_m=evalin('base','C_m');
p1_m=x_m(11:13); p2_m=x_m(14:16); p3_m=x_m(17:19); p4_m=x_m(20:22);%p=1x12, pi=3x1 %i=1:4
%bf_m=x_m(23:25);
%bw_m=x_m(26:28);

delta_r_m=delta_x_m(1:3);
%delta_v_m=delta_x_m(4:6);
delta_fi_m=delta_x_m(7:9); 
%delta_q_m=map_q(delta_fi_m);
delta_p1_m=delta_x_m(10:12); delta_p2_m=delta_x_m(13:15); delta_p3_m=delta_x_m(16:18); delta_p4_m=delta_x_m(19:21);%p=1x12, pi=3x1 %i=1:4
%delta_p=[delta_p1_m; delta_p2_m; delta_p3_m; delta_p4_m];
%delta_bf_m=delta_x_m(22:24);
%delta_bw_m=delta_x_m(25:27);

y=[-C_m*delta_r_m+C_m*delta_p1_m+skew_matrix(C_m*(p1_m-r_m))*delta_fi_m;
   -C_m*delta_r_m+C_m*delta_p2_m+skew_matrix(C_m*(p2_m-r_m))*delta_fi_m;
   -C_m*delta_r_m+C_m*delta_p3_m+skew_matrix(C_m*(p3_m-r_m))*delta_fi_m;
   -C_m*delta_r_m+C_m*delta_p4_m+skew_matrix(C_m*(p4_m-r_m))*delta_fi_m;]

X=S(1:4,1);
Y=S(5:8,1);
Z=S(9:12,1);
s1=[X(1);Y(1);Z(1)];
s2=[X(2);Y(2);Z(2)];
s3=[X(3);Y(3);Z(3)];
s4=[X(4);Y(4);Z(4)];

assignin('base','s1',s1);
assignin('base','s2',s2);
assignin('base','s3',s3);
assignin('base','s4',s4);

pX=P(1:4,1);
pY=P(5:8,1);
pZ=P(9:12,1);

pp1=[pX(1);pY(1);pZ(1)];  %FL
pp2=[pX(2);pY(2);pZ(2)];  %FR
pp3=[pX(3);pY(3);pZ(3)];  %BL
pp4=[pX(4);pY(4);pZ(4)];  %BR

assignin('base','pp1',pp1);
assignin('base','pp2',pp2);
assignin('base','pp3',pp3);
assignin('base','pp4',pp4);

r2=globalPos;
assignin('base','r2',r2);


mat(1,1:3)=rotation(1:3);
mat(2,1:3)=rotation(4:6);
mat(3,1:3)=rotation(7:9);

assignin('base','mat',mat);
%C2= eul2rotm(angles','ZYZ')';
%assignin('base','C2',C2);
assignin('base','quat', q_m);


y3=[s1 - mat*(pp1-r2);
   s2 - mat*(pp2-r2);
   s3 - mat*(pp3-r2);
   s4 - mat*(pp4-r2)];


ypom=[y  y3 ];
%[mat C_m ]
mat-C_m

%y=zeros(12,1);



%%
H=[-C_m zeros(3) skew_matrix(C_m*(p1_m-r_m))  C_m zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p2_m-r_m))  zeros(3) C_m zeros(3) zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p3_m-r_m))  zeros(3) zeros(3) C_m zeros(3) zeros(3) zeros(3);
   -C_m zeros(3) skew_matrix(C_m*(p4_m-r_m))  zeros(3) zeros(3) zeros(3) C_m zeros(3) zeros(3)];

% H=[-mat zeros(3) skew_matrix(mat*(pp1-r2))  mat zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);
%    -mat zeros(3) skew_matrix(mat*(pp2-r2))  zeros(3) mat zeros(3) zeros(3) zeros(3) zeros(3);
%    -mat zeros(3) skew_matrix(mat*(pp3-r2))  zeros(3) zeros(3) mat zeros(3) zeros(3) zeros(3);
%    -mat zeros(3) skew_matrix(mat*(pp4-r2))  zeros(3) zeros(3) zeros(3) mat zeros(3) zeros(3)];

%%

Ra1pom=evalin('base','Ra1pom');
Ra2pom=evalin('base','Ra2pom');
Ra3pom=evalin('base','Ra3pom');
Ra4pom=evalin('base','Ra4pom');

Rs1=evalin('base','Rs1');
Rs2=evalin('base','Rs2');
Rs3=evalin('base','Rs3');
Rs4=evalin('base','Rs4');

if(footZ(1)>-0.254)
    Ra1=(eye(3))*99999;
else
    Ra1=Ra1pom; 

end

if(footZ(2)>-0.254)
    Ra2=(eye(3))*99999;
else
    Ra2=Ra2pom;
end

if(footZ(3)>-0.254)
    Ra3=(eye(3))*99999;
else
    Ra3=Ra3pom; 
 end    

if(footZ(4)>-0.254)
    Ra4=(eye(3))*99999;
else
    Ra4=Ra4pom;
 end



q01=encoders(1:3)';   
Rlkin1=jacobian_dir_kin2(q01);  
R1=double(Rs1+jordan(Rlkin1*Ra1*Rlkin1'));

q02=encoders(4:6)'; 
Rlkin2=jacobian_dir_kin2(q02);  
R2=double(Rs2+jordan(Rlkin2*Ra2*Rlkin2'));

q03=encoders(7:9)';  
Rlkin3=jacobian_dir_kin2(q03);  
R3=double(Rs3+jordan(Rlkin3*Ra3*Rlkin3'));

q04=encoders(10:12)';
Rlkin4=jacobian_dir_kin2(q04);  
R4=double(Rs4+jordan(Rlkin4*Ra4*Rlkin4'));


R=[ R1 zeros(3) zeros(3) zeros(3);
   zeros(3) R2 zeros(3) zeros(3);
   zeros(3) zeros(3) R3 zeros(3); 
   zeros(3) zeros(3) zeros(3) R4];


S=H*P_m*H'+R;
K=P_m*H'*S^-1;
assignin('base','K',K);
assignin('base','H',H);
assignin('base','R',R);
assignin('base','y',y);
Delta_x=K*y;
P_p=(eye(27)-K*H)*P_m;

%[e1 e2 e3]=quat2angle(quatmultiply(angle2quat(angles(1),angles(2),angles(3)),x_m(7:10)'));
%e=[e1 e2 e3];
%delta_fi_p=K(7:9,1)*e;

delta_fi_p=Delta_x(7:9); %delta_fi_p=[0;0;0];  %% PROMJENI
%delta_q_p2=quatmultiply(map_q(delta_fi_p)',q_m')';
delta_q_p=quatmultiply(map_q(delta_fi_p)',q_m')';
%delta_q_p2-delta_q_p

x_state(1:6,1)=x_m(1:6)+Delta_x(1:6); % ili *dt
x_state(7:10,1)=delta_q_p;
x_state(11:28,1)=x_m(11:28)+Delta_x(10:27); % ili *dt

%% saving to matlab workspace

assignin('base','x_state',x_state);
assignin('base','Delta_x',Delta_x);
assignin('base','P_p', P_p);

trag_p=evalin('base','trag_p');
trag_p=[trag_p trace(P_p)];
assignin('base','trag_p',trag_p);


%% dodano

rotm = eul2rotm(delta_fi_p');
newC=rotm*C_m;
assignin('base', 'newC', newC);
C_p=newC;
assignin('base','C_p',C_p);

rotation_p=[C_m C_p];
x_state(7:10,1)=rotm2quat(C_p)';


end