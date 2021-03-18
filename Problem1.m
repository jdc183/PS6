%PS6_q1
%IK for motoman: toolflange w/rt base
%specify the DH parameters:
avec = [0.145,1.150,0.250,0,0,0];
dvec = [0.540,0,0,-1.812,0,-0.1];
alphavec = [3*pi/2,-pi,-pi/2,pi/2,3*pi/2,pi];

%Joint limits
qminvec = [-1.57,-1.4,0,-3,-2,-6];
qmaxvec = [1.57,2.35,1.45,3,2,6];
%Joint velocity limits
qdotmax = [0.75,0.66,1.25,2.8,2.75,3]';

npts = 1000;

y_start=1.6;
p_start= [2.5;-y_start;0.2];

p_des = [2.5;y_start;0.2];
R_des = [ 0.707107,         0,  0.707107;
                 0,        -1,         0;
          0.707107,         0, -0.707107];
      
A_flange_start_wrt_base=eye(4,4);
A_flange_start_wrt_base(1:3,1:3)=R_des;
A_flange_start_wrt_base(1:3,4)=p_start;


[success,soln_vecs] = compute_ik (avec,dvec,alphavec,A_flange_start_wrt_base);
[success,soln_vecs_in_range] = vet_soln_vecs(soln_vecs,qminvec,qmaxvec);
[ndim,nsolns] = size(soln_vecs_in_range);
soln_vecs;
soln_vecs_in_range;

qvec_start =  [-0.58242;-0.37506;0.67150;-0.88764;-0.52531;1.25177];%soln_vecs_in_range(:,3);

[A10,A20,A30,A40,A50,A60] = fk (avec,dvec,alphavec,qvec_start);
fwd_kin_soln = A60;  %redundant check that qvec_start satisfies the desired toolflange pose

J=Jacobian(A10,A20,A30,A40,A50,A60);
  
xyzPath = [linspace(p_start(1),p_des(1),npts);
           linspace(p_start(2),p_des(2),npts);
           linspace(p_start(3),p_des(3),npts)];

qvecs = zeros(6,npts);
qdots = zeros(6,npts-1);
twists = zeros(6,npts-1);
qvecs(:,1) = qvec_start;
times = zeros(1,npts);

for n = 2:npts
    A_flange_des_wrt_base = [R_des xyzPath(:,n); 0 0 0 1];
    [success,soln_vecs] = compute_ik(avec,dvec,alphavec,A_flange_des_wrt_base);
    [success,soln_vecs_in_range] = vet_soln_vecs(soln_vecs,qminvec,qmaxvec);
    [ndim,nsolns] = size(soln_vecs_in_range);
    
    minNorm = 10000.0;%Arbitrary large number
    desSoln = 0;
    for i = 1:nsolns
        if minNorm >= norm(soln_vecs_in_range(:,i) - qvecs(:,n-1))
            minNorm = norm(soln_vecs_in_range(:,i) - qvecs(:,n-1));
            desSoln = i;
        end
    end
    
    qvecs(:,n) = soln_vecs_in_range(:,desSoln);
    
    [A10,A20,A30,A40,A50,A60] = fk (avec,dvec,alphavec,qvecs(:,n));
    J=Jacobian(A10,A20,A30,A40,A50,A60);
    
    %dqj = J\[0;2*y_start/npts;0;0;0;0];
    
    dq = qvecs(:,n) - qvecs(:,n-1);
    dt = max(abs(dq)./qdotmax);
    qdots(:,n-1) = dq./dt;
    
    [A10,A20,A30,A40,A50,A60] = fk (avec,dvec,alphavec,qvecs(:,n));
    J=Jacobian(A10,A20,A30,A40,A50,A60);
    twists(:,n-1) = J*qdots(:,n-1);
    
    times(n) = times(n-1) + dt;
end

figure
plot(times,qvecs);
title("Joint space values vs time");
xlabel("Time (s)");
ylabel("Joint space value (radians)")
legend('q_1','q_2','q_3','q_4','q_5','q_6')

figure
qdots = diff(qvecs,1,2)./diff([times;times;times;times;times;times],1,2);
plot(times(1:npts-1),qdots);
title("Joint Space Velocities vs Time");
xlabel("Time (s)");
ylabel("Joint Space Velocity (radians/second)")
legend('q_1''','q_2''','q_3''','q_4''','q_5''','q_6''')

figure
plot(times(1:npts-1),twists);
title("Task Space Velocities (Twist) vs Time");
xlabel("Time (s)");
ylabel("Twist (meters/second or radians/second)")
legend('v_x','v_y','v_z','w_x','w_y','w_z')
