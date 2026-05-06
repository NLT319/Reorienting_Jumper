function ref_traj = generate_tracking_lqr(jumper, jump_theta, K, optimizer)
% OUTPUT: ref_traj - struct for tracking_lqr_controller
    if nargin < 4
        optimizer = 'fmincon';
    end
    
    %% Run Trajectory Optimization
    switch lower(optimizer)
        case 'fmincon'
            soln = optimizeLandingTraj_fmincon(jump_theta, jumper);
        case 'optimtraj'
            if ~exist('optimTraj', 'file')
                error('OptimTraj not found');
            end
            soln = optimizeLandingTraj_optimtraj(jump_theta, jumper);
        otherwise
            error('Invalid optimizer choice');
    end
    
    if soln.info.exitFlag <= 0
        error('Optimization failed exit flag: %d', soln.info.exitFlag);
    end
    
    % Reference Trajectory
    T_ref = soln.grid.time;
    X_ref = soln.grid.state;  % 2 x N: [theta; dtheta]
    U_ref = soln.grid.control;  % 1 x N
    
    % Calculate reference wheel speed
    Jt = jumper.Jb + jumper.mw*jumper.Lbw^2;
    H0 = jumper.Jw * jumper.dphi0 + Jt * X_ref(2,1);
    dphi_w_ref = (H0 - Jt * X_ref(2,:)) / jumper.Jw;
    
    % Struct
    ref_traj = struct();
    ref_traj.time = T_ref;
    ref_traj.theta = X_ref(1,:);
    ref_traj.dtheta = X_ref(2,:);
    ref_traj.u_ff = U_ref;
    ref_traj.dphi_wheel = dphi_w_ref;
    ref_traj.K = K;  % Store K in the struct for convenience
    
    save('tracking_lqr_reference.mat', 'ref_traj', 'K', 'jumper');
end