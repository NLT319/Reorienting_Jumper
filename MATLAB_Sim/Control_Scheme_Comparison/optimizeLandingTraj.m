function soln = optimizeLandingTraj(thb0, jumper)

phi_w_max = 100 * 2*pi/60; % ~10 rad/s
Jw = jumper.Jw;
Jt = jumper.Jb + jumper.mw*jumper.Lbw^2;
H0 = Jw * jumper.dphi0;
[t_land, ~] = landing_time(jumper);
%t_noControl = t_land/10; % seconds with no control


problem = struct();

% dynamics (double integrator)
problem.func.dynamics = @(t,x,u)[x(2, :); -u./Jt];

% cost
problem.func.pathObj = @(t,x,u) u.^2;

% fixed time
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low   = t_land - 0.1;
problem.bounds.finalTime.upp   = t_land + 0.1;

problem.func.pathCst = @(t,x,u) deal(H0 - Jt*x(2)/Jw , []);
problem.bounds.pathCst.low = -phi_w_max*10;
problem.bounds.pathCst.upp =  phi_w_max*10;

% initial state
problem.bounds.initialState.low = [thb0; 0];
problem.bounds.initialState.upp = [thb0; 0];

%final state
problem.bounds.finalState.low = [pi - thb0 - 0.05; -0.05];
problem.bounds.finalState.upp = [pi - thb0 + 0.05; 0.05];
%problem.func.bndObj = @(t0,x0,tf,xf) 10*xf(2)^2;

% control bounds
problem.bounds.control.low = -100;%-jumper.umax;
problem.bounds.control.upp =  100;%jumper.umax;
% 
% problem.func.pathCst = @(t,x,u) deal(u .* (t <= t_noControl), []);
% problem.bounds.pathCst.low = -1e-6; % lower/upper bound for path constraint
% problem.bounds.pathCst.upp = 1e-6;

% guess
problem.guess.time = [0 t_land];
problem.guess.state = [thb0, pi-thb0; 0, 0];
problem.guess.control = [0 0];


problem.options.method = 'trapezoid';
problem.options.nGrid = 80;
problem.options.verbose = 2;

%problem.options.nlpOpt = optimset('Display', 'iter', 'MaxIter',2000);
%'TolFun',1e-4, 'TolX',1e-6, 'TolCon',1e-6,
soln = optimTraj(problem);

end