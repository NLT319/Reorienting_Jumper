function u = tracking_lqr_controller(t, X, K, X_ref, U_ref, p)

    % Extract reference trajectory
    ref_traj = X_ref;
    
    % Interpolate reference at current time
    if t <= ref_traj.time(1)
        theta_ref = ref_traj.theta(1);
        dtheta_ref = ref_traj.dtheta(1);
        u_ff = ref_traj.u_ff(1);
    elseif t >= ref_traj.time(end)
        theta_ref = ref_traj.theta(end);
        dtheta_ref = ref_traj.dtheta(end);
        u_ff = ref_traj.u_ff(end);
    else
        theta_ref = interp1(ref_traj.time, ref_traj.theta, t);
        dtheta_ref = interp1(ref_traj.time, ref_traj.dtheta, t);
        u_ff = interp1(ref_traj.time, ref_traj.u_ff, t);
    end
    
    % Tracking control law: u = u_ff - K*(x - x_ref)
    X_ref_now = [theta_ref; dtheta_ref];
    error = X - X_ref_now;
    
    % Use K from ref_traj if available, otherwise use passed K
    if isfield(ref_traj, 'K')
        K_use = ref_traj.K;
    else
        K_use = K;
    end
    
    u = u_ff - K_use * error;
    
    % Saturate to actuator limits
    u = max(min(u, p.umax), -p.umax);
end