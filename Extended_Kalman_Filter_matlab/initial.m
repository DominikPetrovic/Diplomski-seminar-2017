dt=0.01;
syms q1 q2 q3 q4

%parametri robota
a1=0; a2=0.1002; a3=0.20425; a4=0.01908;
d1=0; d2=0.053;  d3=0; d4=0;
q4=29.04*pi/180;
save params

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


