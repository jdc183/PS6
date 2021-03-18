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
%% @deftypefn {} {@var{retval} =} fk (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-01

%specialized for 6DOF; thetas must be in DH coords
function [A10,A20,A30,A40,A50,A60] = fk (avec,dvec,alphavec,thetavec)
  A10 =  a_of_dh (avec(1),dvec(1),alphavec(1),thetavec(1));
  A21 = a_of_dh(avec(2),dvec(2),alphavec(2),thetavec(2));
  A20 = A10*A21;

  A32 = a_of_dh(avec(3),dvec(3),alphavec(3),thetavec(3));
  A30 = A20*A32;
  A43 = a_of_dh(avec(4),dvec(4),alphavec(4),thetavec(4));
  A40 = A30*A43;
  A54 = a_of_dh(avec(5),dvec(5),alphavec(5),thetavec(5));
  A50 = A40*A54;
  A65 = a_of_dh(avec(6),dvec(6),alphavec(6),thetavec(6));
  A60 = A50*A65;
end
