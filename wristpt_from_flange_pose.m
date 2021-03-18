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
%% @deftypefn {} {@var{retval} =} wristpt_from_flange_pose (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07

function [wrist_pt] = wristpt_from_flange_pose (A_flange_wrt_base,d6)
  z_des_flange = A_flange_wrt_base(1:3,3);
  p_des_flange = A_flange_wrt_base(1:3,4);
  wrist_pt = p_des_flange+d6*z_des_flange; %don't know why + instead of -

end
