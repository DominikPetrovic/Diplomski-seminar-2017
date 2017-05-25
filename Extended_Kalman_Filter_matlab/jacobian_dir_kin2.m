function Jac=jacobian_dir_kin2(q0)

syms q1 q2 q3 %q4

q4=29.03*pi/180;

l1=0.18;
l2=0.09;
l3=0.05;
l4=0.06;
l5=0.1;
l6=0.15;
ld6 = 0.12;

Proj_z = l5*cos(q2) + l6*cos(q2 + q3); %projekcija noge na z os

fi = (pi - q1) / 2;
d = 2 * Proj_z*sin(q1 / 2);
dz = d*cos(fi);

x = -l5*sin(q2) - l6*sin(q2 + q3);
y = -sin(q1)*Proj_z;		%pozitivan zakret motora pomièe nogu u negativnom smjeru y osi
z = dz - Proj_z;
  
w=[x;y;z];

Jac=jacobian(w,[q1 q3 q3 ]); 

q44=29.03*pi/180;

Jac=vpa(subs(Jac,[q1 q2 q3 ], [q0]));

end