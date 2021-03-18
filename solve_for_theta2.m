%% Copyright (C) 2021 Wyatt
%% 
%% This program is free software; you can redistribute it and/or modify it
%% under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%% 
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%% 
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% -*- texinfo -*- 
%% @deftypefn {} {@var{retval} =} solve_for_theta2 (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07

function [success, q2_soln] = solve_for_theta2 (w_wrt_1, r_goal,L_humerus,L_forearm)
  q2_soln=0;
  success=true;
     beta = atan2(w_wrt_1(2), w_wrt_1(1));    
     acos_arg = (L_humerus * L_humerus + r_goal * r_goal - L_forearm * L_forearm) / (2.0 * r_goal * L_humerus);
    if (abs(acos_arg > 1.0)) 
        success=false; % ROS_WARN("hey!  logic err acos_arg = %f", acos_arg);
        return %false;
    end
    gamma = acos(acos_arg);
    %//ROS_INFO("beta,gamma = %f %f",beta,gamma);
    %//q2_solns[0] = M_PI / 2.0 - beta - gamma; //%elbow up--choose this soln;
    %q2_soln = beta + gamma %pi -gamma + beta; %//beta+gamma-M_PI/2.0; //%elbow up--choose this soln;  
    q2_soln = beta -gamma;
end
