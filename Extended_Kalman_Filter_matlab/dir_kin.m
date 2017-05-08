function w=dir_kin(q,foot)

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
    
           
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

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


w=[T(1:3,4); T(1:3,3)];



end