
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-08

function [success] = joints_in_range (q_vec, qmin_vec,qmax_vec)
  success=true;
  for i=1:6
    if (q_vec(i)<qmin_vec(i))
      success=false;
      return
    end
    if (q_vec(i)>qmax_vec(i))
      success=false;
      return
    end
    end
end
