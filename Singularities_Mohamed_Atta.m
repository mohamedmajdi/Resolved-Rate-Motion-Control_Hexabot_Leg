syms q1 q2 q3 L1 L2 L3;

Jacobian=[-L1*sin(q1)-L2*cos(q2)*sin(q1)-L3*cos(q2+q3)*sin(q1) , -L2*cos(q1)*sin(q2)-L3*cos(q1)*sin(q2+q3) , -L3*cos(q1)*sin(q2+q3);
          L1*cos(q1)+L2*cos(q1)*cos(q2)+L3*cos(q1)*cos(q2+q3) , -L2*sin(q1)*sin(q2)-L3*sin(q1)*sin(q2+q3) , -L3*sin(q1)*sin(q2+q3);
                                                            0 , L2*cos(q2)+L3*cos(q2+q3) ,  L3*cos(q2+q3)];

solve(det(Jacobian) == 0,[q1 q2 q3])