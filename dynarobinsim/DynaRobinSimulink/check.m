dt=2;
x_state_0=pom;
prediction_0(x_state_0, dt);

for i=1:5
update_step_sim(x_m, delta_x_m, dt, P_m, encoders);
w_IMU=zeros(3,1);
f_IMU=zeros(3,1);
prediction_step_sim(x, delta_x, dt, w_IMU, f_IMU, P_p);
end