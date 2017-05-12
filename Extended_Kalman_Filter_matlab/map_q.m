function mapped_q=map_q(fi)

%   kvaternion mapped_q je 4x1
%   pomak kuta Dfi je 3x1

if(fi(1)==0 && fi(2)==0 && fi(3)==0)
   pom1=[0;0;0];
else
pom1=sin(0.5*norm(fi))*fi/norm(fi);
end
pom2=cos(0.5*norm(fi));
mapped_q=[pom1;pom2];

end
