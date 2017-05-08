function mapped_omega=map_omega(omega)

wx=omega(1);
wy=omega(2);
wz=omega(3);

mapped_omega=[0 wz -wy wx;
              -wz 0 wx wy;
              wy -wx 0 wz; 
              -wx -wy -wz 0];

end
