syms t1 t2 t3 t4 t5 t6 t7
T01 = [cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0; 0 0 1 .333; 0 0 0 1];
T12 = [cos(t2) -sin(t2) 0 0; 0 0 1 0; -sin(t2) -cos(t2) 0 0; 0 0 0 1];
T23 = [cos(t3) -sin(t3) 0 0; 0 0 -1 -.316; sin(t3) cos(t3) 0 0; 0 0 0 1];
T34 = [cos(t4) -sin(t4) 0 .0825; 0 0 -1 0; sin(t4) cos(t4) 0 0; 0 0 0 1];
T45 = [cos(t5) -sin(t5) 0 -.0825; 0 0 1 .384; -sin(t5) -cos(t5) 0 0; 0 0 0 1];
T56 = [cos(t6) -sin(t6) 0 0; 0 0 -1 0; sin(t6) cos(t6) 0 0; 0 0 0 1];
T67 = [cos(t7) -sin(t7) 0 .088; 0 0 -1 0; sin(t7) cos(t7) 0 0; 0 0 0 1];
T7F = [1 0 0 0; 0 1 0 0; 0 0 1 .107; 0 0 0 1];
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;
T07 = T06*T67;
T0F = T07*T7F;
R01 = T01(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);
R04 = T04(1:3,1:3);
R05 = T05(1:3,1:3);
R06 = T06(1:3,1:3);
R07 = T07(1:3,1:3);
R0F = T0F(1:3,1:3);

P0F = T0F(1:3,4);
TopJacobian = sym(zeros(3,7));
for i = 1:3
for j = 1:7
    sysName = strcat("t",num2str(j));
    temp = diff(P0F(i),sysName);
    TopJacobian(i,j) = temp;
end
end

RotationList = {R01,R02,R03,R04,R05,R06,R07,R0F};
Z = [0;0;1];
BottomJacobian = sym(zeros(3,7));
for i = 1:7
temp = RotationList{i}*Z;
BottomJacobian(:,i) = temp;
end
Jacobian = [TopJacobian;BottomJacobian]

%disp(TopJacobian);