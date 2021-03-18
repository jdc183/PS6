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
%% @deftypefn {} {@var{retval} =} solve_for_theta3 (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07

function [success, q3_soln] = solve_for_theta3 (w_wrt_1a, r_goal,L_humerus,L_forearm,A2,L3)
    %goal_radius = r_goal
    acos_arg = (L_humerus*L_humerus + L_forearm*L_forearm - r_goal*r_goal)/(2.0*L_humerus*L_forearm);
    q3_soln=0;
    success=true;
    if (abs(acos_arg>1.0)) 
        success=false;
        return  %false;
    end
    %//ROS_INFO("solve_for_theta3: r_goal = %f",r_goal);
    eta = acos(acos_arg); %//equiv elbow angle of simple 2DOF arm   
    cos_eta = cos(eta);
    cos_pi_minus_eta = cos(pi-eta);
    sin_pi_minus_eta = sin(pi-eta);
    %//ROS_INFO("eta, phi_elbow = %f, %f",eta,phi_elbow);
    %//first solution is elbow "up"
    %rtest = sqrt((L_humerus + L_forearm*cos(pi-eta))^2 + (L_forearm*sin(pi-eta))^2);
    phi_elbow=acos((A2*A2+L_forearm*L_forearm-L3*L3)/(2.0*A2*L_forearm));

    %q3_soln=  phi_elbow +eta -pi; 
    q3_soln = phi_elbow +eta-pi;
    %pause
end
