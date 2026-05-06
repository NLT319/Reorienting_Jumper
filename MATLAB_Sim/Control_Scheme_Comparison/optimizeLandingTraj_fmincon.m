function soln = optimizeLandingTraj_fmincon(thb0, jumper)
% Trajectory optimization using direct collocation + fmincon

    % Parameters
    phi_w_max = 100 * 2*pi/60; % max wheel speed
    Jw = jumper.Jw;
    Jt = jumper.Jb + jumper.mw*jumper.Lbw^2;
    H0 = Jw * jumper.dphi0;
    
    % Calculate correct landing time based on TIP hitting ground
    [t_land, ~] = landing_time_tip(jumper);
    
    % Motor limits (asymmetric!)
    u_accel_max = jumper.umax;
    u_brake_max = 2 * jumper.umax;  % braking is stronger
    %u_accel_max = 
    
    % Discretization
    N = 60;  % number of time steps
    dt = t_land / (N-1);
    t_grid = linspace(0, t_land, N);
    
    % Decision Variables
    % z = [theta(1:N), dtheta(1:N), u(1:N)]
    % Total: 3*N variables
    
    n_vars = 3*N;
    
    % Better guess improves convergence
    theta_guess = linspace(thb0, jumper.final_theta, N);
    dtheta_guess = (jumper.final_theta - thb0) / t_land * ones(1, N);
    u_guess = 0.2 * jumper.umax * sin(pi * t_grid / t_land);
    
    z0 = [theta_guess, dtheta_guess, u_guess];
    
    % Theta bounds
    theta_lb = -2*pi * ones(1, N);
    theta_ub =  2*pi * ones(1, N);
    
    % Angular velocity bounds
    dtheta_lb = -20 * ones(1, N);
    dtheta_ub =  20 * ones(1, N);
    
    % Control bounds
    u_lb = -u_brake_max * ones(1, N);
    u_ub =  u_accel_max * ones(1, N);
    
    lb = [theta_lb, dtheta_lb, u_lb];
    ub = [theta_ub, dtheta_ub, u_ub];
    
    % Boundary conditions
    % theta(1) = thb0
    % dtheta(1) = 0
    % ~ no control until t_start?
    % theta(N) = final_theta
    % dtheta(N) = 0

    
    n_eq = 4;% + t_start_idx;
    Aeq = zeros(n_eq, n_vars);
    beq = zeros(n_eq, 1);
    
    % theta(1) = thb0
    Aeq(1, 1) = 1;
    beq(1) = thb0;
    
    % dtheta(1) = 0
    Aeq(2, N+1) = 1;
    beq(2) = 0;
    
    % theta(N) = final_theta
    Aeq(3, N) = 1;
    beq(3) = jumper.final_theta;
    
    % dtheta(N) = 0
    Aeq(4, 2*N) = 1;
    beq(4) = 0;
% 
%     for k = 1:t_start_idx
%     Aeq(4+k, 2*N+k) = 1;
%     beq(4+k) = 0;
%     end
    
    % Nonlinear Constraints
    % Collocation constraints enforcing dynamics
    % wheel speed limit
    % motor limits
    
    nonlcon = @(z) collocationConstraints(z, N, dt, Jt, H0, Jw, phi_w_max, u_accel_max, u_brake_max);
    
    % Cost Function
    % Minimize control effort (integral of u^2)
    costfun = @(z) costFunction(z, N, dt);
    
    % Solver Options
    options = optimoptions('fmincon', ...
        'Display', 'iter-detailed', ...
        'Algorithm', 'sqp', ...  % SQP works well for trajectory optimization
        'MaxFunctionEvaluations', 10000000, ...
        'MaxIterations', 3000, ...
        'OptimalityTolerance', 1e-6, ...
        'ConstraintTolerance', 1e-6, ...
        'StepTolerance', 1e-10, ...
        'FiniteDifferenceType', 'central', ...
        'UseParallel', false);
    
    %% Solve
    fprintf('\nTrajectory Optimization (fmincon)\n');
    fprintf('Grid points: %d\n', N);
    fprintf('Time step:   %.4f s\n', dt);
    fprintf('Flight time: %.3f s\n', t_land);
    fprintf('Initial angle: %.2f deg → Final angle: %.2f deg\n', ...
        thb0*180/pi, jumper.final_theta*180/pi);
    
    tic;
    [z_opt, fval, exitflag, output] = fmincon(costfun, z0, [], [], Aeq, beq, lb, ub, nonlcon, options);
    solve_time = toc;
    
    fprintf('\nOptimization completed in %.2f seconds\n', solve_time);
    fprintf('Exit flag: %d (%s)\n', exitflag, getExitFlagMessage(exitflag));
    fprintf('Final cost: %.6f\n', fval);
    
    %% Solution
    theta_opt = z_opt(1:N);
    dtheta_opt = z_opt(N+1:2*N);
    u_opt = z_opt(2*N+1:3*N);
    
    % Calculate wheel speed
    dphi_wheel = (H0 - Jt * dtheta_opt) / Jw;
    
    % Store solution
    soln.grid.time = t_grid;
    soln.grid.state = [theta_opt; dtheta_opt];
    soln.grid.control = u_opt;
    soln.grid.dphi_wheel = dphi_wheel;
    soln.info.exitFlag = exitflag;
    soln.info.objVal = fval;
    soln.info.output = output;
    soln.info.solveTime = solve_time;
    
    fprintf('Final angle error:    %.4f deg\n', (theta_opt(end) - jumper.final_theta)*180/pi);
    fprintf('Final velocity error: %.4f rad/s\n', dtheta_opt(end));
    fprintf('Max wheel speed:      %.2f rad/s (limit: %.2f)\n', max(abs(dphi_wheel)), phi_w_max);
    fprintf('Max control:          %.4f Nm (accel limit: %.2f, brake limit: %.2f)\n', ...
        max(abs(u_opt)), u_accel_max, u_brake_max);
end

%% Functions
function J = costFunction(z, N, dt)
    u = z(2*N+1:3*N);
    % Trapezoidal integration of u^2
    J = dt * (0.5*u(1)^2 + sum(u(2:N-1).^2) + 0.5*u(N)^2);
end

function [c, ceq] = collocationConstraints(z, N, dt, Jt, H0, Jw, phi_w_max, u_accel_max, u_brake_max)
    
    theta = z(1:N);
    dtheta = z(N+1:2*N);
    u = z(2*N+1:3*N);
    
    % Equality constraints using collocation
    % d(theta)/dt = dtheta
    % d(dtheta)/dt = -u/Jt
    
    n_dynamics = 2*(N-1);
    ceq = zeros(n_dynamics, 1);
    
    idx = 1;
    for k = 1:N-1
        % Trapezoidal theta
        ceq(idx) = theta(k+1) - theta(k) - dt * 0.5 * (dtheta(k) + dtheta(k+1));
        idx = idx + 1;
        
        % Trapezoidal dtheta
        ddtheta_k = -u(k) / Jt;
        ddtheta_kp1 = -u(k+1) / Jt;
        ceq(idx) = dtheta(k+1) - dtheta(k) - dt * 0.5 * (ddtheta_k + ddtheta_kp1);
        idx = idx + 1;
    end
    
    % Inequality Constraints
    c = [];
    
    %wheel speed limit is abs(phi_wheel) <= phi_w_max
    dphi_wheel = (H0 - Jt * dtheta) / Jw;
    c_wheel_upper = dphi_wheel - phi_w_max;      % dphi <= phi_max
    c_wheel_lower = -phi_w_max - dphi_wheel;     % -phi_max <= dphi

    c = [c_wheel_upper'; c_wheel_lower'];
end

function msg = getExitFlagMessage(exitflag)
% interpret exit flag
    switch exitflag
        case 1
            msg = 'First-order optimality conditions satisfied';
        case 2
            msg = 'Change in x too small';
        case 0
            msg = 'Too many iterations or function evaluations';
        case -1
            msg = 'Stopped by output/plot function';
        case -2
            msg = 'No feasible point found';
        otherwise
            msg = 'Unknown exit condition';
    end
end

function [t_land, x_land] = landing_time_tip(jumper)
    % For a given final angle, height correction for when the foot touches ground    
    theta_final = jumper.final_theta;
    % y_tip = y_com - (Lb/2)*sin(theta_final) = 0
    % y_com_landing = (Lb/2)*sin(theta_final)
    y_com_required = (jumper.Lb/2) * sin(theta_final);
    
    % ballistic trajectory
    a = -0.5 * jumper.g;
    b = jumper.vy0;
    c = jumper.y0 - y_com_required;
    
    t_roots = roots([a, b, c]);
    t_land = max(t_roots(t_roots > 0));  % take positive root
    
    % Horizontal position at landing
    x_land = jumper.vx0 * t_land;
    
    % Verify tip is at ground
    y_com_at_landing = jumper.y0 + jumper.vy0*t_land - 0.5*jumper.g*t_land^2;
    y_tip_check = y_com_at_landing - (jumper.Lb/2)*sin(theta_final);
    
    if abs(y_tip_check) > 1e-6
        warning('Tip height error: %.6f m (should be ~0)', y_tip_check);
    end
end
