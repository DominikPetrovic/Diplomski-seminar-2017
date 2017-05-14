function Jac=jacobian_dir_kin(q0,foot)

load params

if (~(strcmp(foot ,'BR') || strcmp(foot ,'BL') || strcmp(foot ,'FL') || strcmp(foot ,'FR')))
        msgID = 'MYFUN:incorrectSize';
        msg = 'Foot must be assign as FR, FL, BR or BL';
        ME=MException(msgID,msg);
        throw(ME)
end
      
if(strcmp(foot ,'FL') || strcmp(foot ,'BR'))
    d2=d2;
else if (strcmp(foot ,'FR') || strcmp(foot ,'BL'))
    d2=-d2;
    end
end

syms q1 q2 q3 q4


T(1,1)=cos(q1)*cos(q2+q3+q4);
T(1,2)=-cos(q1)*sin(q2+q3+q4);
T(1,3)=sin(q1);
T(1,4)=d2*sin(q1)+cos(q1)*(a2*cos(q2)+a3*cos(q2+q3)+a4*cos(q2+q3+q4));

T(2,1)=sin(q1)*cos(q2+q3+q4);
T(2,2)=-sin(q1)*sin(q2+q3+q4);
T(2,3)=-cos(q1);
T(2,4)=-d2*cos(q1)+sin(q1)*(a2*cos(q2)+a3*cos(q2+q3)+a4*cos(q2+q3+q4));

T(3,1)=sin(q2+q3+q4);
T(3,2)=cos(q2+q3+q4);
T(3,3)=0;
T(3,4)=a2*sin(q2)+a3*sin(q2+q3)+a4*sin(q2+q3+q4);

T(4,1)=0;
T(4,2)=0;
T(4,3)=0;
T(4,4)=1;

                                                            % PROVJERITI ????
Jac=jacobian(T(1:3,4),[q1 q3 q3 q4]); 

q44=29.03*pi/180;

Jac=vpa(subs(Jac,[q1 q2 q3 q4], [q0 q44]));

end