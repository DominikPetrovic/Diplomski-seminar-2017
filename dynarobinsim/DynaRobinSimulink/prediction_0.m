function prediction_0(x, dt)

delta_x=zeros(27,1);
f_IMU=zeros(3,1);
w_IMU=zeros(3,1);
P_p=zeros(27,27);
footZ=[-0.26 -0.26 -0.26 -0.26];
prediction_step_sim(x, delta_x, dt, f_IMU, w_IMU, P_p, footZ);
end