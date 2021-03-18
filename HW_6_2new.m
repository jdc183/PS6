a = [0.145, 1.150, 0.250, 0, 0, 0];
d = [0.540, 0, 0, -1.812, 0, -0.1];
alpha = [3*pi/2, -pi, -pi/2, pi/2, 3*pi/2, pi];
theta = [q1, q2, q3, q4, q5, q6];
q = [0.87674, -0.78611, 0.21930, 0.16801, 1.68849, 4.65091];


a2 = [0.145, 1.15-0.5];
d3 = [0.540, 0, 0, -1];


p1 = A10(1:3,4);
p2 = A20(1:3,4);
p3 = A30(1:3,4);
p4 = A40(1:3,4);
p5 = A50(1:3,4);
p6 = A60(1:3,4);

z0 = [0; 0; 1];
z1 = A10(1:3,3);
z2 = A20(1:3,3);
z3 = A30(1:3,3);
z4 = A40(1:3,3);
z5 = A50(1:3,3);
z6 = A60(1:3,3);

[A10,A20,A30,A40,A50,A60] = fk(a,d,alpha,q);
%% solve Jacobian of mass 2
Ac2 = A(a2,d,alpha,theta,1,2);
Pc2 = Ac2(1:3,4);   
    
for i = 1:3
    Pc2(i) = double(subs(Pc2(i)));
end
Pc2 = double(Pc2);
Jp2 = [cross(z0, Pc2), cross(z1, (Pc2-p1)), zeros(3,4)];
Jo2 = [z0, z1, zeros(3,4)];
Jc2 = [Jp2; Jo2];

%% solve Jacobian of mass 3
Ac3 = A(a,d3,alpha,theta,1,4);
Pc3 = Ac3(1:3,4);   
    
for i = 1:3
    Pc3(i) = double(subs(Pc3(i)));
end
Pc3 = double(Pc3);

Jp3 = [cross(z0, Pc3), cross(z1, (Pc3-p1)), cross(z2, (Pc3-p2)), zeros(3,3)];
Jo3 = [z0, z1, z2, zeros(3,3)];
Jc3 = [Jp3; Jo3];

%% motor torques required to against gravity
m2 = 200;
m3 = 200;
g = 10;
tc2 = Jc2' * [0; 0; m2*g; 0; 0; 0];
tc3 = Jc3' * [0; 0; m3*g; 0; 0; 0];
tg = tc2 + tc3;
disp('motor torques required to against gravity')
disp(tg)

%% motor torques required to against gravity and weight
weight = 20;
J = Jacobian (A10,A20,A30,A40,A50,A60);
tw = J' * [0; 0; weight*g; 0; 0; 0];
t_total = tc2 + tc3 + tw;
disp('motor torques required to against gravity and weight')
disp(t_total)


%% function DHtoA
function y = A(a,d,alpha,theta,m,n)
    y = eye(4);
 
    for i = m:n
        temp = [ cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
              sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
              0,              sin(alpha(i)),                cos(alpha(i)),               d(i);
              0,              0,                             0,                            1];
       y = y * temp;
    end
end