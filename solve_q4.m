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
%% @deftypefn {} {@var{retval} =} solve_q4 (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-07

function [q4_soln] = solve_q4 (avec,dvec,alphavec,q1,q2,q3,R_des)
  A10 =  a_of_dh (avec(1),dvec(1),alphavec(1),q1);
  A21 = a_of_dh(avec(2),dvec(2),alphavec(2),q2);
  A20 = A10*A21;

  A32 = a_of_dh(avec(3),dvec(3),alphavec(3),q3);
  A30 = A20*A32;
    
    r3 = A30(1:3,1); 
    g3 = A30(1:3,2);    
    b3 = A30(1:3,3); 
    b_des = R_des(1:3,3);
    r_des = R_des(1:3,2);
    b4 = -cross(b3,b_des);
    b4_norm = norm(b4);
    b4_hat = b4/b4_norm;
    cq4 = b4_hat'*g3;
    sq4 = b4_hat'*r3;
    q4_soln = atan2(sq4,cq4);
    q4_soln = -q4_soln+pi;
    
end
