dt=0.01;
x_state_0=pom;
prediction_0(x_state_0, dt);

encoders=[0.0000; 0.4735; -0.8632; 
          0.0000; 0.4735; -0.8632; 
          0.0000; -0.4734; 0.8632;
          0.0000; -0.4734; 0.8632];
       
%encoders=[ 3.0796; -2.8001; 1.9164;  %FL
%         -0.0620; -2.8310; 1.9164; %FR
%          -0.0620; -2.8310;  1.9164; %BL
%          3.0796;  0.1793; -2.0002;]; %BR
      

update_step_sim(x_m, delta_x_m, dt, P_m, encoders, footZ);

for i=1:5
w_IMU=zeros(3,1);
f_IMU=zeros(3,1);
prediction_step_sim(x_state, delta_x, dt, w_IMU, f_IMU, P_p);
footZ=[-0.3; -0.3; -0.3; -0.3];
update_step_sim(x_m, delta_x_m, dt, P_m, encoders, footZ);
end