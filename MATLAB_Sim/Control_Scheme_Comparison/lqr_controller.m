function u = lqr_controller(t, x, K, X_ref, ~, p)
%x_ref = [p.final_theta; 0; 0; 0];
[t_land, x_land] = landing_time(p);
% t_start = t_land/5;
% if(t < t_start)
%     u = 0;
% else
x_ref = X_ref;
u = - K * (x - x_ref);
u = min(max(u, -p.umax), p.umax);
end
%end