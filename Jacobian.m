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
%% @deftypefn {} {@var{retval} =} Jacobian (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-01

function [J] = Jacobian (A10,A20,A30,A40,A50,A60)
J = zeros(6,6); 

J_ang = zeros(3,6);
J_ang(:,1) = [0;0;1];
J_ang(:,2) =   A10(1:3,3);
J_ang(:,3) = A20(1:3,3);
J_ang(:,4) = A30(1:3,3);
J_ang(:,5) = A40(1:3,3);
J_ang(:,6) = A50(1:3,3);
J(4:6,:) = J_ang;

%translational part: compute r_hand-r_n and extract z_n
r_hand = A60(1:3,4);

jnt=1;
z = J_ang(:,jnt);
r_n = [0;0;0]; %
J_pos(1:3,jnt) = cross(z,r_hand-r_n);

jnt=2;
z = J_ang(:,jnt);
r_n = A10(1:3,4);
J_pos(1:3,jnt) = cross(z,r_hand-r_n);

jnt=3;
z = J_ang(:,jnt);
r_n = A20(1:3,4);
J_pos(1:3,jnt) = cross(z,r_hand-r_n);

jnt=4;
z = J_ang(:,jnt);
r_n = A30(1:3,4);
J_pos(1:3,jnt) = cross(z,r_hand-r_n);

jnt=5;
z = J_ang(:,jnt);
r_n = A40(1:3,4);
J_pos(1:3,jnt) = cross(z,r_hand-r_n);

jnt=6;
z = J_ang(:,jnt);
r_n = A50(1:3,4);
J_pos(1:3,jnt) = cross(z,r_hand-r_n);

J(1:3,:)=J_pos;
end
