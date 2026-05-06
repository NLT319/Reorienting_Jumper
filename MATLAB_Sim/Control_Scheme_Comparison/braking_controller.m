function u = braking_controller(t, x, K, X_ref, U_ref, p)
% phiw tracking clutch braking controller
% somewhat hacky, but ode45 is annoying since it backtracks and breaks my
% systems

theta  = x(1);
dtheta = x(2);

Jt = p.Jb + p.mw*p.Lbw^2;
Jw = p.Jw;
theta_f = p.final_theta;
T_brake = 0.1; % how long to hold the braking torque for
H0 = Jw * p.bdphi0;
[t_land, ~] = landing_time_tip(p);

% time left
t_remain = t_land - t;
if t_remain <= 0
    u = 0;
    return
end

phi_w = (H0 - Jt*dtheta) / Jw;

% post break body vel
dtheta_post = (H0 + Jt*dtheta) / Jt;

% brake start time
t_brake = t_land - (theta_f - theta) / dtheta_post;

% keep track of braking state
persistent braking
if(isempty(braking) || t <= 1e-3)
    braking = false;
end

u = 0;

% start braking
if ~braking && t >= t_brake
    braking = true;
    % Finite brake duration

    % Exact torque to stop wheel in T_brake
    %u_brake = -Jw * phi_w / T_brake;
end

if(braking)
% apply braking torque for set time
    if abs(phi_w) > 1e-4 % check when wheel is stopped 
%     if t <= t_brake + T_brake
        u = -Jw*phi_w/T_brake;
        %disp(phi_w*u)

    else
        braking = false;
        u = 0;
    end
end % this feels like writing embedded code for microcontrollers
% end
end

function [t_land, x_land] = landing_time_tip(jumper)
    % For a given final angle, height correction for when the foot touches ground    
    theta_final = jumper.final_theta;
    % y_tip = y_com - (Lb/2)*sin(theta_final) = 0
    % y_com_landing = (Lb/2)*sin(theta_final)
    y_com_required = (jumper.Lb) * sin(theta_final);
    
    % ballistic trajectory
    a = -0.5 * jumper.g;
    b = jumper.vy0;
    c = jumper.y0 - y_com_required;
    
    t_roots = roots([a, b, c]);
    t_land = max(t_roots(t_roots > 0));  %positive root
    
    % Horizontal position at landing
    x_land = jumper.vx0 * t_land;
    
    % Verify tip is at ground
    y_com_at_landing = jumper.y0 + jumper.vy0*t_land - 0.5*jumper.g*t_land^2;
    y_tip_check = y_com_at_landing - (jumper.Lb)*sin(theta_final);
    
    if abs(y_tip_check) > 1e-6
        warning('Tip height error: %.6f m (should be ~0)', y_tip_check);
    end
end
