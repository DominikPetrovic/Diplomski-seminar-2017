function q=inv_kin(w,foot)

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

x=w(1); y=w(2); z=w(3);

%helper variable for calculating q1
fi=atan2(x/sqrt(x^2+y^2),-y/sqrt(x^2+y^2));

q11=fi+acos(d2/sqrt(x^2+y^2));
q12=fi-acos(d2/sqrt(x^2+y^2));

%helper variables for calculating q3
l=sqrt(a3^2+a4^2+2*a3*a4*cos(q4));
ksi=atan2(a4/l*sin(q4),(a3+a4*cos(q4))/l);

%q3xy, x=1/2 for q11/q12; y=1/2 for +/-
q311=acos((((x-d2*sin(q11))/cos(q11))^2+z^2-a2^2-l^2)/(2*a2*l))-ksi;
q312=-acos((((x-d2*sin(q11))/cos(q11))^2+z^2-a2^2-l^2)/(2*a2*l))-ksi;
q321=acos((((x-d2*sin(q12))/cos(q12))^2+z^2-a2^2-l^2)/(2*a2*l))-ksi;
q322=-acos((((x-d2*sin(q12))/cos(q12))^2+z^2-a2^2-l^2)/(2*a2*l))-ksi;

%helper variables for calculating q2
q2sin11=(z-(l*sin(q311+ksi))/(a2+l*cos(q311+ksi))*(x-d2*sin(q11)/cos(q11)))/(a2+l*cos(q311+ksi)+(l*sin(q311+ksi))^2/(a2+l*cos(q311+ksi)));
q2sin12=(z-(l*sin(q312+ksi))/(a2+l*cos(q312+ksi))*(x-d2*sin(q11)/cos(q11)))/(a2+l*cos(q312+ksi)+(l*sin(q312+ksi))^2/(a2+l*cos(q312+ksi)));
q2sin21=(z-(l*sin(q321+ksi))/(a2+l*cos(q321+ksi))*(x-d2*sin(q12)/cos(q12)))/(a2+l*cos(q321+ksi)+(l*sin(q321+ksi))^2/(a2+l*cos(q321+ksi)));
q2sin22=(z-(l*sin(q322+ksi))/(a2+l*cos(q322+ksi))*(x-d2*sin(q12)/cos(q12)))/(a2+l*cos(q322+ksi)+(l*sin(q322+ksi))^2/(a2+l*cos(q322+ksi)));

q2cos11=((x-d2*sin(q11)/cos(q11))+(l*sin(q311+ksi))*z/(a2+l*cos(q311+ksi)))/(a2+l*cos(q311+ksi)+(l*sin(q311+ksi))^2/(a2+l*cos(q311+ksi)));
q2cos12=((x-d2*sin(q11)/cos(q11))+(l*sin(q312+ksi))*z/(a2+l*cos(q312+ksi)))/(a2+l*cos(q312+ksi)+(l*sin(q312+ksi))^2/(a2+l*cos(q312+ksi)));
q2cos21=((x-d2*sin(q12)/cos(q12))+(l*sin(q321+ksi))*z/(a2+l*cos(q321+ksi)))/(a2+l*cos(q321+ksi)+(l*sin(q321+ksi))^2/(a2+l*cos(q321+ksi)));
q2cos22=((x-d2*sin(q12)/cos(q12))+(l*sin(q322+ksi))*z/(a2+l*cos(q322+ksi)))/(a2+l*cos(q322+ksi)+(l*sin(q322+ksi))^2/(a2+l*cos(q322+ksi)));


q211=atan2(q2sin11,q2cos11);
q212=atan2(q2sin12,q2cos12);
q221=atan2(q2sin21,q2cos21);
q222=atan2(q2sin22,q2cos22);

q2=[q211; q212; q221; q222];

q=[q11 q211 q311 q4;
   q11 q212 q312 q4;
   q12 q221 q321 q4;
   q12 q222 q322 q4];


end






