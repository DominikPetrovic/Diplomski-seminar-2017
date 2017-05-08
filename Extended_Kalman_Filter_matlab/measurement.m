function s= measurement(q,p,r,q0)
%vector p(i) and s(i)= 3x1
%vector p and s=1x12
%quaternion q=4x1
C=quat2rotm(q');
p1=p(1:3)';
p2=p(4:6)';
p3=p(7:9)';
p4=p(10:12)';

%ni= measurement noise quantity with covariance matrix Ri
%Ri=Rs+JIkini*Ra*Jkini' ???????
                        
                        % jakobijan?
Ra=ones(4);              % šta s tim???
Rs=ones(3);           % takoðer????
Rlkin=jacobian_dir_kin(q0,'FR');        % dodati Jakobijan za svaku nogu. Koji q0 uzeti ????
%covariance matrix for ni
R=double(Rs+Rlkin*Ra*Rlkin');    % kako dodati matricu kovarijanci šumu ?? 
n=mvnrnd([0,0,0],R);               %da li je to to???
n1=wgn(3,1,0);n2=wgn(3,1,0);n3=wgn(3,1,0);n4=wgn(3,1,0);
s1=C*(p1-r)+n1;
s2=C*(p2-r)+n2;
s3=C*(p3-r)+n3;
s4=C*(p4-r)+n4;

s=[s1 s2 s3 s4];
end