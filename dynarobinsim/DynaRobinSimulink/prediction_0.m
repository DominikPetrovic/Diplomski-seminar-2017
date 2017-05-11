function prediction_0(x, dt)

delta_x=zeros(27,1);
f_IMU=rand(3,1);
w_IMU=rand(3,1);
P_p=rand(27,27);
prediction_step_sim(x, delta_x, dt, f_IMU, w_IMU, P_p);
end