

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07
%this function is specialized for the Motoman GP20HL
%provide the DH parameter constants and the desired toolflange pose with respect to the 
%base frame, A_flange_des_wrt_base
%value of "success" will be "true" if there is at least one IK solution
%soln_vecs will contain 6DOF joint-value solutions in columns
%NOTE: these have not be vetted for conforming to joint limits;
%follow up by calling vet_soln_vecs() to trim the solution candidates to only reachable candidates

function [success,soln_vecs] = compute_ik (avec,dvec,alphavec,A_flange_des_wrt_base)
d6 = dvec(6);
wrist_pt = wristpt_from_flange_pose (A_flange_des_wrt_base,d6);
R_des = A_flange_des_wrt_base(1:3,1:3);
[success,soln] = compute_q123 (avec,dvec,alphavec, wrist_pt);
if (~success) 
  return
end
%soln_vec_123=zeros(6,1);
q1=soln(1);
q2=soln(2);
q3=soln(3);
soln_vecs = add_wrist_solns(avec,dvec,alphavec,q1,q2,q3,R_des);

end
