

%% Author: Wyatt <wsn@ava>
%% Created: 2021-03-08

function [found_a_viable_soln,soln_vecs_in_range] = vet_soln_vecs (soln_vecs,qminvec,qmaxvec)
  [ndim,nsolns]=size(soln_vecs);
  found_a_viable_soln=false;
  soln_vecs_in_range=[];
  for i=1:nsolns
    if (joints_in_range (soln_vecs(:,i), qminvec,qmaxvec)  )
      %success=true;
      %soln_vecs_in_range=[soln_vecs_in_range,soln_vecs(:,i)];
      if (~found_a_viable_soln)
        found_a_viable_soln=true;
        soln_vecs_in_range=soln_vecs(:,i); %first vector in viable soln vec
      else  
        soln_vecs_in_range=[soln_vecs_in_range,soln_vecs(:,i)]; %append another soln
      end
    
    end %end if jnts in range
  end %end loop through solns

end
