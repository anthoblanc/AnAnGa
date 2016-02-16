n1 = VarName3(1:3:end);
n2 = VarName4(1:3:end);
n3 = VarName5(1:3:end);
figure(1)
plot3(n1,n2,-n3)
xlabel('x');ylabel('y');zlabel('height')
title('Desired Path')
hold on
c1 = VarName3(2:3:end);
c2 = VarName4(2:3:end);
c3 = VarName5(2:3:end);
figure(2)
plot3(c1,c2,-c3)
xlabel('x');ylabel('y');zlabel('height')
title('Real Trajectory')
hold on
l1 = VarName3(3:3:end);
l2 = VarName4(3:3:end);
l3 = VarName5(3:3:end);
figure(3)
plot3(l1,l2,-l3)
% axis([0 inf 0 inf 0 inf])
xlabel('x');ylabel('y');zlabel('height')
title('L-vector')
hold on