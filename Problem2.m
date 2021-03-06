%PS6_q2
%IK for motoman: toolflange w/rt base
%specify the DH parameters:
avec = [0.145,1.150,0.250,0,0,0];
dvec = [0.540,0,0,-1.812,0,-0.1];
alphavec = [3*pi/2,-pi,-pi/2,pi/2,3*pi/2,pi];

q = [0.87674,  -0.78611,   0.21930,   0.16801,   1.68849,   4.65091]';
[A10,A20,A30,A40,A50,A60] = fk (avec,dvec,alphavec,q);

cg1 = [1;0;0];%Filler value to prevent cross product with zero vector
cg2 = [-0.5;0;0];
cg3 = [0;0;-1];
cg4 = [1;0;0];%Filler value to prevent cross product with zero vector
cg5 = [1;0;0];%Filler value to prevent cross product with zero vector
cg6 = [0;0;0];

[J1,J2,J3,J4,J5,J6] = cg_amputee_jacobians(A10,A20,A30,A40,A50,A60,cg1,cg2,cg3,cg4,cg5,cg6);

w2 = [0;0;200*9.8;0;0;0];
w3 = [0;0;200*9.8;0;0;0];
w6 = [0;0;20*9.8;0;0;0];

unloadedTorques = J2'*w2 + J3'*w3
loadedTorques = unloadedTorques + J6'*w6

%Compute amputee Jacobians for each link
function [J1,J2,J3,J4,J5,J6] = cg_amputee_jacobians(A1,A2,A3,A4,A5,A6,cg1,cg2,cg3,cg4,cg5,cg6)
    R1 = A1(1:3,1:3);
    R2 = A2(1:3,1:3);
    R3 = A3(1:3,1:3);
    R4 = A4(1:3,1:3);
    R5 = A5(1:3,1:3);
    R6 = A6(1:3,1:3);
    
    % Z-axes wrt base frame
    z0 = [0;0;1];
    z1 = A1(1:3,3);
    z2 = A2(1:3,3);
    z3 = A3(1:3,3);
    z4 = A4(1:3,3);
    z5 = A5(1:3,3);
    
    % Tool flange wrt each joint frame
    o60 = A6(1:3,4) + R6*cg6;
    o61 = o60-A1(1:3,4);
    o62 = o60-A2(1:3,4);
    o63 = o60-A3(1:3,4);
    o64 = o60-A4(1:3,4);
    o65 = o60-A5(1:3,4);
    
    J6 = [cross(z0,o60),cross(z1,o61),cross(z2,o62),cross(z3,o63),cross(z4,o64),cross(z5,o65);
                     z0,           z1,           z2,           z3,           z4,          z5];
    
    % cg5 wrt each joint frame
    o50 = A5(1:3,4) + R5*cg5;
    o51 = o50-A1(1:3,4);
    o52 = o50-A2(1:3,4);
    o53 = o50-A3(1:3,4);
    o54 = o50-A4(1:3,4);
    
    J5 = [cross(z0,o50),cross(z1,o51),cross(z2,o52),cross(z3,o53),cross(z4,o54),zeros(3,1);
                     z0,           z1,           z2,           z3,           z4,zeros(3,1)];
                 
    % cg4 wrt each joint frame
    o40 = A4(1:3,4) + R4*cg4;
    o41 = o40-A1(1:3,4);
    o42 = o40-A2(1:3,4);
    o43 = o40-A3(1:3,4);
    
    J4 = [cross(z0,o40),cross(z1,o41),cross(z2,o42),cross(z3,o43),zeros(3,2);
                     z0,           z1,           z2,           z3,zeros(3,2)];
    
    % cg3 wrt each joint frame
    o30 = A3(1:3,4) + R3*cg3;
%     A4_0_m3 = A3;
%     A4_0_m3(1:3,4) = o30
    o31 = o30-A1(1:3,4);
    o32 = o30-A2(1:3,4);
    
    J3 = [cross(z0,o30),cross(z1,o31),cross(z2,o32),zeros(3,3);
                     z0,           z1,           z2,zeros(3,3)];
                 
    % cg2 wrt each joint frame
    o20 = A2(1:3,4) + R2*cg2;
    o21 = o20-A1(1:3,4);
    
    J2 = [cross(z0,o20),cross(z1,o21),zeros(3,4);
                     z0,           z1,zeros(3,4)];
    
    % cg1 wrt each joint frame
    o10 = A1(1:3,4) + R1*cg1;
    
    J1 = [cross(z0,o10),zeros(3,5);
                     z0,zeros(3,5)];
    
end

