function prediction_0(x, dt)
%%
Qf=rand(3,3)*0.01; %Qf=eye(3)*0.01; 
Qbf=rand(3,3)*0.01; %Qbf=eye(3)*0.01;
Qw=rand(3,3)*0.01; %Qw=eye(3)*0.01;
Qbw=rand(3,3)*0.01; %Qbw=eye(3)*0.01;
Qp=rand(3,3)*0.01; %Qp=eye(3)*0.01;

%Qf=zeros(3); Qbf=zeros(3); Qw=zeros(3); Qbw=zeros(3); Qp=zeros(3);

Qf=Qf*Qf'; Qbf=Qbf*Qbf'; Qw=Qw*Qw'; Qbw=Qbw*Qbw'; Qp=Qp*Qp';

assignin('base','Qf',Qf);
assignin('base','Qbf',Qbf);
assignin('base','Qw',Qw);
assignin('base','Qbw',Qbw);
assignin('base','Qp',Qp);

%%
Rs1=(eye(3))*0.01; %Rs1=wgn(3,3,0);
Rs2=(eye(3))*0.01; %Rs2=wgn(3,3,0);
Rs3=(eye(3))*0.01; %Rs3=wgn(3,3,0);
Rs4=(eye(3))*0.01; %Rs4=wgn(3,3,0);

 
for i=1:3
    for j=1:3
        if(i==j)
            Rs1(i,j)=1; Rs2(i,j)=1; Rs3(i,j)=1; Rs4(i,j)=1;
        end
    end
end 	

Rs1=eye(3)*0.1; Rs2=eye(3)*0.1; Rs3=eye(3)*0.1; Rs4=eye(3)*0.1; 

assignin('base','Rs1',Rs1);
assignin('base','Rs2',Rs2);
assignin('base','Rs3',Rs3);
assignin('base','Rs4',Rs4);

Ra1pom=(eye(3))*0.01; %Ra1pom=wgn(3,3,0);
Ra2pom=(eye(3))*0.01; %Ra2pom=wgn(3,3,0);
Ra3pom=(eye(3))*0.01; %Ra3pom=wgn(3,3,0);
Ra4pom=(eye(3))*0.01; %Ra4pom=wgn(3,3,0);


for i=1:3
    for j=1:3
        if(i==j)
            Ra1pom(i,j)=1; Ra2pom(i,j)=1; Ra3pom(i,j)=1; Ra4pom(i,j)=1;
        end
    end
end 

Ra1pom=(eye(3))*0.1; Ra2pom=(eye(3))*0.1; Ra3pom=(eye(3))*0.1; Ra4pom=(eye(3))*0.1; 
assignin('base','Ra1pom',Ra1pom);
assignin('base','Ra2pom',Ra2pom);
assignin('base','Ra3pom',Ra3pom);
assignin('base','Ra4pom',Ra4pom);

%%
Qp_1=[1 0.1 0.1; 
      0.1 1 0.1; 
      0.1 0.1 1];
  
Qp_2=[1 0.1 0.1; 
      0.1 1 0.1; 
      0.1 0.1 1];
  
Qp_3=[1 0.1 0.1; 
      0.1 1 0.1; 
      0.1 0.1 1];
  
Qp_4=[1 0.1 0.1; 
      0.1 1 0.1; 
      0.1 0.1 1];
  
assignin('base','Qp_1', Qp_1);
assignin('base','Qp_2', Qp_2);
assignin('base','Qp_3', Qp_3);
assignin('base','Qp_4', Qp_4);

wp_1=mvnrnd([0 0 0],Qp_1)'*0.01; % ???
wp_2=mvnrnd([0 0 0],Qp_2)'*0.01; % ??? 
wp_3=mvnrnd([0 0 0],Qp_3)'*0.01; % ??? 
wp_4=mvnrnd([0 0 0],Qp_4)'*0.01; % ??? 

%wp_1=zeros(3,1);
%wp_2=zeros(3,1);
%wp_3=zeros(3,1);
%wp_4=zeros(3,1);

assignin('base','wp_1', wp_1);
assignin('base','wp_2', wp_2);
assignin('base','wp_3', wp_3);
assignin('base','wp_4', wp_4);

%%
ww=[0 0 0]'; wbw=[0 0 0]'; wf=[0 0 0]'; wbf=[0 0 0]'; %% PROMJENI 
%ww=(rand(3,1)-0.5)*0.01; wbw=(rand(3,1)-0.5)*0.01; wf=(rand(3,1)-0.5)*0.01; wbf=(rand(3,1)-0.5)*0.01; %% PROMJENI 


assignin('base','ww',ww);
assignin('base','wbw',wbw);
assignin('base','wf',wf);
assignin('base','wbf',wbf);


%delta_x_0=(rand(27,1)-0.5);
delta_x_0=zeros(27,1);
delta_x_0(1:21)=0.1;
delta_x_0(22:27)=0.01;
f_IMU=zeros(3,1);
w_IMU=zeros(3,1);

%P_p=(rand(27,27)-0.5)*0.1;
%P_p=eye(27,27);
%P_p=ones(27,27);
P_p=zeros(27,27);

prediction_step_sim(x, delta_x_0, dt, f_IMU, w_IMU, P_p);
end