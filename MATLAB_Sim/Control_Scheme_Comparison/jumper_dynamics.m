function [dX, u] = jumper_dynamics(t, X, controller, K, X_ref, U_ref, p)
% Pinned so only rotation dynamics
% State: X = [thb; phiw; dthb; dphiw]

    % Extract states
    thb = X(1); dthb = X(2); %phiw = X(2); dthb = X(3); dphiw = X(4);
    
    % set controller
    if(~isa(controller,'function_handle'))
        u = controller;
    else
        u = controller(t, X, K, X_ref, U_ref, p);
    end

    % Mass matrix
    %M_pinned = [p.Jb + p.mw*p.Lbw^2, 0; 0, p.Jw];
    J_total = p.Jb + p.mw*p.Lbw^2;
    
    % Torque vector
    %Tau = [-u - p.Lbw*p.g*p.mw*cos(thb); u];  
    %Tau = [-u; u];
    
    % Accelerations
    %d2q = M_pinned\Tau;
    d2q = -u/J_total;
    
    % State derivatives
    dX = [dthb; d2q]; % dq2 is 2x1
end

