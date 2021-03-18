%PS6_q1
%IK for motoman: toolflange w/rt base
clear all
%specify the DH parameters:
avec = [0.145,1.150,0.250,0,0,0];
dvec = [0.540,0,0,-1.812,0,-0.1];
alphavec = [3*pi/2,-pi,-pi/2,pi/2,3*pi/2,pi];
qminvec = [-1.57,-1.4,0,-3,-2,-6]
qmaxvec = [1.57,2.35,1.45,3,2,6]
qdotmax = [0.75,0.66,1.25,2.8,2.75,3]

y_start=1.6
p_des = [2.5;-y_start;0.2]
R_des = [ 0.707107,         0,  0.707107;
        0,        -1,         0;
 0.707107,         0, -0.707107]
A_flange_des_wrt_base=eye(4,4);
A_flange_des_wrt_base(1:3,1:3)=R_des;
A_flange_des_wrt_base(1:3,4)=p_des
format short


[success,soln_vecs] = compute_ik (avec,dvec,alphavec,A_flange_des_wrt_base);
[success,soln_vecs_in_range] = vet_soln_vecs(soln_vecs,qminvec,qmaxvec);
[ndim,nsolns] = size(soln_vecs_in_range)
soln_vecs
soln_vecs_in_range

qvec_start =soln_vecs_in_range(:,3)'
[A10,A20,A30,A40,A50,A60] = fk (avec,dvec,alphavec,qvec_start);
fwd_kin_soln = A60  %redundant check that qvec_start satisfies the desired toolflange pose

  J=Jacobian (A10,A20,A30,A40,A50,A60);
