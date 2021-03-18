
%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07
%function to compute solution for q1,q2, q3 for Motoman GP20 given a desired wrist point
%this solution forces pi/2>q1>-pi/2 and forces "elbow up", i.e. q3>0, so q1,q2,q3 soln is unique 
%(if it exists, and if not at a singularity)

function [success,soln] = compute_q123 (avec,dvec,alphavec, wrist_pt_des)
  soln=[];
  success=true;
  q1a = atan2(wrist_pt_des(2),wrist_pt_des(1));
  if (q1a>pi/2) 
    q1a = q1a-pi;
  end
  if (q1a<-pi/2) 
    q1a = q1a+pi;
    end
   A10 = a_of_dh (avec(1),dvec(1),alphavec(1),q1a);
   p1_wrt_0 = A10(1:3,4);
   R10 = A10(1:3,1:3);
   w_wrt_1 = R10'*wrist_pt_des -R10'*p1_wrt_0;
   r_goal = sqrt(w_wrt_1(1)*w_wrt_1(1)+w_wrt_1(2)*w_wrt_1(2));
   L_humerus = avec(2);
   L3 = abs(dvec(4));
   A2 = avec(3);
   L_forearm = sqrt(A2*A2+L3*L3);
   if (r_goal>=L_humerus+L_forearm)
     success=false;
     return
   end
   %display('computeq123 line 31; pausing')
   %pause
   %solve for the elbow angle, q3:
   [success, q3_soln] = solve_for_theta3(w_wrt_1, r_goal,L_humerus,L_forearm,A2,L3);
   if (~success)
     return
   end
   %display('computeq123 line 38; pausing')
   %pause   
   %solve for shoulder angle, q2:
   %w_wrt_1
   [success, q2_soln] = solve_for_theta2(w_wrt_1, r_goal,L_humerus,L_forearm);
   if (~success)
     return;
     end

   soln = [q1a;q2_soln;q3_soln];  
end
