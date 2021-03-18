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
%% @deftypefn {} {@var{retval} =} add_wrist_solns (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07

function [soln_vecs] = add_wrist_solns (avec,dvec,alphavec,q1,q2,q3,R_des)
soln_vecs = [q1;q2;q3;0;0;0];
q4 = solve_q4 (avec,dvec,alphavec,q1,q2,q3,R_des);
soln_vecs(4)=q4;
q5 = solve_q5 (avec,dvec,alphavec,q1,q2,q3,q4,R_des);
soln_vecs(5)=q5;
q6 = solve_q6 (avec,dvec,alphavec,q1,q2,q3,q4,q5,R_des);
soln_vecs(6)=q6;
soln_vecb=soln_vecs;
%consider options of toolflange + or - 360deg
if (soln_vecb(6)>0) 
  soln_vecb(6) = soln_vecb(6)-2*pi;
else
  soln_vecb(6) = soln_vecb(6)+2*pi;
end
soln_vecs=[soln_vecs,soln_vecb];
%consider alt soln w/ wrist bent opposite direction
soln_vec2=soln_vecs(:,1);
soln_vec2(4)=soln_vec2(4)+pi;
if (soln_vec2(4)>pi)
  soln_vec2(4)=soln_vec2(4)-2*pi;
  end
soln_vec2(6)=soln_vec2(6)+pi;
if (soln_vec2(6)>pi)
  soln_vec2(6)=soln_vec2(6)-2*pi;
  end
soln_vec2(5)= -soln_vecs(5);
soln_vecs = [soln_vecs,soln_vec2];
soln_vec2b = soln_vec2;
%consider adding or subtracting 2pi from toolflange rotation
if (soln_vec2b(6)>0) 
  soln_vec2b(6) = soln_vec2b(6)-2*pi;
else
  soln_vec2b(6) = soln_vec2b(6)+2*pi;
end
soln_vecs=[soln_vecs,soln_vec2b];  

end
