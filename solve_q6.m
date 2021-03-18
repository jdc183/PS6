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
%% @deftypefn {} {@var{retval} =} solve_q6 (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07

function [q6_soln] = solve_q6 (avec,dvec,alphavec,q1,q2,q3,q4,q5,R_des)
    %solve for q6
  % the following is redundant w/ solve_q4; could combine these to make more efficient
  A10 =  a_of_dh (avec(1),dvec(1),alphavec(1),q1);
  A21 = a_of_dh(avec(2),dvec(2),alphavec(2),q2);
  A20 = A10*A21;
  A32 = a_of_dh(avec(3),dvec(3),alphavec(3),q3);
  A30 = A20*A32;
  A43 = a_of_dh(avec(4),dvec(4),alphavec(4),q4);
  A40 = A30*A43;
  A54 = a_of_dh(avec(5),dvec(5),alphavec(5),q5);
  A50 = A40*A54;
  b_des = R_des(1:3,3);
  r_des = R_des(1:3,1);
  r5 = A50(1:3,1); %A05.col(0).head(3);
  g5 = A50(1:3,2); %A05.col(1).head(3);   
        
  cq6= r_des'*r5; %.dot(-n5);
  sq6= r_des'*g5; %.dot(-t5);
  q6_soln =atan2(sq6, cq6);
end
