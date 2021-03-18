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
%% @deftypefn {} {@var{retval} =} a_of_dh (@var{input1}, @var{input2})
%%
%% @seealso{}
%% @end deftypefn

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-01

function [A] = a_of_dh (a,d,alpha,theta)
cq = cos(theta);
sq = sin(theta);
sa = sin(alpha);
ca = cos(alpha);
A = eye(4,4);
A(1,1)=cq;
A(1,2) = -sq*ca;
A(1,3) = sq*sa;
A(2,1)=sq;
A(2,2)=cq*ca;
A(2,3)= -cq*sa;
A(3,2) = sa;
A(3,3) = ca;
A(1,4) = a*cq;
A(2,4) = a*sq;
A(3,4) =d;

end
