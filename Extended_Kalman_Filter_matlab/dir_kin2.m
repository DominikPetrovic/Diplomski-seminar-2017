function w=dir_kin2(q)
l1=0.18;
l2=0.09;
l3=0.05;
l4=0.06;
l5=0.1;
l6=0.15;
ld6 = 0.12;

q1=q(1);
q2=q(2);
q3=q(3);

Proj_z = l5*cos(q2) + l6*cos(q2 + q3); %projekcija noge na z os

fi = (pi - q1) / 2;
d = 2 * Proj_z*sin(q1 / 2);
dz = d*cos(fi);

w(1) = -l5*sin(q2) - l6*sin(q2 + q3);
w(2) = -sin(q1)*Proj_z;		%pozitivan zakret motora pomièe nogu u negativnom smjeru y osi
w(3) = dz - Proj_z;
  
end