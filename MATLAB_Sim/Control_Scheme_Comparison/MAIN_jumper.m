%% set params
clc; close all; clear;
mw_diff = [0.1 0.5 0.9]; % 

% Add in 

%%%%%%%% Block of changing parameters %%%%%%%%%

% mb = 0.16
% mw = 0.02
% Lb = .00235
% wb = x
% Rw = 0.16


y0 = 0; % Start height (relative to goal)
dphi0 = 0; % initial wheel velocity (8)
v0 = 25; % calculate velocities
jump_theta = 45*pi/180; % convert to rads
g = 9.81; % gravity
umax = 2;
bdphi0 = 2; % brakign initial velocity

mb = 0.1;  % the "leg" (aka "body")
mw = mw_diff(1);%0.5;  % the inertia "wheel"
Lb = 1;
wb = 0.08; % width of body
Jb = (1/12)*mb*Lb^2; % wrt com
Lbw = 0; % from COM of body to COM of wheel
Rw = 0.2;
Jw = (1/2)*mw*Rw^2; % wrt com

mb = 0.16
mw = 0.02
%Lb = .00235
Rw = 0.16

% cost for active flywheel for the motor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pack into struct
jumper.mb = mb;
jumper.mw = mw;
jumper.Lb = Lb;
jumper.wb = wb;
jumper.Jb = Jb;
jumper.Lbw = Lbw;
jumper.Rw = Rw;
jumper.Jw =Jw;
jumper.g = g;
jumper.umax = umax;
jumper.dphi0 = dphi0;
jumper.bdphi0 = bdphi0;


vx0 = v0*cos(jump_theta);
vy0 = v0*sin(jump_theta);
E_wheel = Jw*dphi0;
jumper.E_wheel = E_wheel;
jumper.y0 = y0 + Lb*sin(jump_theta);
jumper.vx0 = vx0;
jumper.vy0 = vy0;
jumper.final_theta = pi - jump_theta;


[t_land, x_land] = landing_time(jumper);

%[A, B, K] = jumper_lin(jumper);

%A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
J_total = Jb + mw*Lbw^2;
%B = [0; 0; -1/(J_total); 1/Jb];
% uncontrollable -> reduce states
% x1 = thetab
% x2 = dthetab
% don't even care about the speed or angle of the wheel

A = [0 1; 0 0];
B = [0; -1/J_total];

% LQR 
Q = diag([10, 1]); % 10, 1
R = 1; % 10
K = lqr(A, B, Q, R)

X_ref = [jumper.final_theta; 0];


screen = get(0, 'ScreenSize');
sw = screen(3);
sh = screen(4);

fig_pos = [sw*0.005, sh*0.245, sw*0.61, sh*0.65];

%%
% state X = [theta_body; phi_wheel; dtheta_body; dphi_wheel]
%X0 = [jump_theta; 0; 0; dphi0];
X0 = [jump_theta; 0];
tspan = [0, 5];

lands = @(t,X) landing_event(t, X, jumper);

% ODE options with event detection
opts = odeset('Events', lands);

controllers = @lqr_controller;%, @braking_controller};
 % for looping over ics
 i = 1;
%for i = 1:length(controllers)
    %for j = 1:length(mw_diff)
%jumper.mw = mw_diff(3)
%jumper.jump_angle = jump_angle_error(i);
controller = controllers
% testing no control
%controller = @no_control;
%controller = @pd_controller;
%controller = @lqr_controller;
%controller = @braking_controller;
[tsim, xsim, te, xe, ie] = ode45(@(t, X) jumper_dynamics(t, X, controller, K, X_ref, [], jumper), tspan, X0, opts);
uout = 0*tsim;
for n=1:length(tsim)
    [~,uout(n)] = jumper_dynamics(tsim(n),xsim(n,:)', controller, K, X_ref, [], jumper);
end

% calc wheel speed
Jt = jumper.Jb + jumper.mw*jumper.Lbw^2;
H0 = E_wheel + Jt * xsim(1,2); % initial angular momentum
dphi_w = (H0 - Jt*xsim(:,2))/jumper.Jw;

% Store total effort
w_rel = dphi_w - xsim(:, 2); % relative angular velocity of wheel
E_total = cumtrapz(tsim, uout.*w_rel);
effort = cumtrapz(tsim, abs(uout.*w_rel));


TotE(i) = effort(end);
dphi_w_lqr = dphi_w;

xend(i) = xsim(end, 1);
yend(i) = xsim(end, 2);

%end
% end
%% Animate
figure('Name', 'LQR Controller', 'Position', fig_pos);%[100 100 1200 800]);
subplot(3, 2, [1,2]);
draw_jumper(tsim, xsim, jumper, 1, dphi_w);



%% Plot
%figure(1); clf
%subplot(2,2,1); 
subplot(3, 2, 3); hold on;
yyaxis left;
plot(tsim, xsim(:,1)*180/pi)
ylabel('Angle (deg)')
%xlabel('Time (s)')
%title('Body Orientation')
ylim([0.9*min(180/pi*xsim(:,1)), max(180/pi*xsim(:,1))*1.1])

grid on

% Body angular velocity
%subplot(2,2,2)
yyaxis right;
plot(tsim, xsim(:,2))
ylabel('Angular Velocity (rad/s)')
%xlabel('Time (s)')
title('Body States')
legend(["Body Angle"; "Angular Velocity"], "Location", "northwest")
ylim([min(xsim(:,2)) - 0.2, max(xsim(:,2))*1.1])
grid on

% Control torque
subplot(3,2,4)
plot(tsim, uout)
ylabel('Torque (N*m)')
%xlabel('Time (s)')
title('Input Torque')
grid on


%figure(2); clf

subplot(3,2,5)
plot(tsim, dphi_w)
ylabel('Speed (rad/s)')
xlabel('Time (s)')
title('Calculated Flywheel Speed')
grid on

dmomentum_LQR = dphi_w_lqr;

subplot(3,2,6); hold on;
Ewheel = 0.5 * jumper.Jw * dphi_w.^2;
w_rel = dphi_w - xsim(:, 2); % relative angular velocity of wheel
E_total = cumtrapz(tsim, uout.*w_rel);
effort = cumtrapz(tsim, abs(uout.*w_rel));
plot(tsim, E_total)
plot(tsim, effort)
ylabel('Energy (J)')
xlabel('Time (s)')
title('Flywheel Kinetic Energy')
legend(["Energy used"; "Total Energy used"], "location", "northwest")
grid on
pause(2)

%% Plot error smear

figure()
%scatter(xend, yend)
%scatter(["LQR", "Braking"], TotE, 'xr')
% xlabel("Normalized error in jump angle")
% ylabel("Total effort used in control (J)")
TotE(2) = effort(end) + 0.5 * jumper.Jw * jumper.dphi0^2;%H0;
bar(TotE)
ylabel("Total Energy Used Inair (J)");
xlabel("LQR                  Brake")
title("Reaction Wheel m_w = 0.1")
ylim([0 0.5])


%% Braking controller also gets shown
clear global uout_log tsim_log
global uout_log tsim_log
uout_log = [];
tsim_log = [];
jumper.umax = 1000;

X0 = [jump_theta; 0];
tspan = [0, 5];

outputFcn = @(t, X, flag) captureControl(t, X, flag, controller, K, X_ref, jumper);

% Setup options with output function
opts = odeset('Events', lands, 'OutputFcn', outputFcn);

lands = @(t,X) landing_event(t, X, jumper);

opts = odeset('Events', lands, 'OutputFcn', outputFcn);

controllers = @braking_controller;%, @tracking_lqr_controller};
 % for looping over ics
%for i = 1:length(controllers)
    %for j = 1:length(mw_diff)
%jumper.mw = mw_diff(3)
%jumper.jump_angle = jump_angle_error(i);
controller = controllers
i = 2;
% testing no control
%controller = @no_control;
%controller = @pd_controller;
%controller = @lqr_controller;
%controller = @braking_controller;
[tsim, xsim, te, xe, ie] = ode45(@(t, X) jumper_dynamics(t, X, controller, K, X_ref, [], jumper), tspan, X0, opts);
uout = 0*tsim;
for n=1:length(tsim)
    [~,uout(n)] = jumper_dynamics(tsim(n),xsim(n,:)', controller, K, X_ref, [], jumper);
end

if ~isempty(tsim_log)
    uout = interp1(tsim_log, uout_log, tsim, 'previous', 0);
else
    warning('No control data captured!');
    uout = zeros(size(tsim));
end

% calc wheel speed
Jt = jumper.Jb + jumper.mw*jumper.Lbw^2;
H0 = E_wheel + Jt * xsim(1,2); % initial angular momentum
dphi_w = jumper.bdphi0 + (H0 - Jt*xsim(:,2))/jumper.Jw;
%dphi_w = (H0 - Jt*xsim(:,2))/jumper.Jw;

% Store total effort
% w_rel = dphi_w - xsim(:, 2); % relative angular velocity of wheel
% E_total = cumtrapz(tsim, uout.*w_rel);
% effort = cumtrapz(tsim, abs(uout.*w_rel));


%TotE(i) = effort(end);
TotE(i) = E_wheel;
dphi_w_brake = dphi_w;
xend(i) = xsim(end, 1);
yend(i) = xsim(end, 2);

   % end
% end
%% Animate
figure('Name', 'Braking Controller', 'Position', fig_pos);%[100 100 1200 800]);
subplot(3, 1, 1);
draw_jumper(tsim, xsim, jumper, 1, dphi_w);



%% Plot
%figure(1); clf
%subplot(2,2,1); 
subplot(3, 1, 2); hold on;
yyaxis left;
plot(tsim, xsim(:,1)*180/pi)
ylabel('Angle (deg)')
%xlabel('Time (s)')
%title('Body Orientation')
ylim([0.9*min(180/pi*xsim(:,1)), max(180/pi*xsim(:,1))*1.1])

grid on

% Body angular velocity
%subplot(2,2,2)
yyaxis right;
plot(tsim, xsim(:,2))
ylabel('Angular Velocity (rad/s)')
%xlabel('Time (s)')
title('Body States')
legend(["Body Angle"; "Angular Velocity"], "Location", "northwest")
ylim([min(xsim(:,2)) - 0.2, max(xsim(:,2))*1.1])
grid on

% % Control torque
% subplot(3,2,4)
% plot(tsim, uout)
% ylabel('Torque (N*m)')
% %xlabel('Time (s)')
% title('Input Torque')
% grid on


%figure(2); clf

subplot(3,1,3)
plot(tsim, dphi_w)
ylabel('Speed (rad/s)')
xlabel('Time (s)')
title('Calculated Flywheel Speed')
grid on

% subplot(3,2,6); hold on;
Ewheel = 0.5 * jumper.Jw * dphi_w.^2;
w_rel = dphi_w - xsim(:, 2); % relative angular velocity of wheel
E_total = cumtrapz(tsim, uout.*w_rel);
effort = cumtrapz(tsim, abs(uout.*w_rel));
% plot(tsim, E_total)
% plot(tsim, effort)
% ylabel('Energy (J)')
% xlabel('Time (s)')
% title('Flywheel Kinetic Energy')
% legend(["Energy used"; "Total Energy used"], "location", "northwest")
% grid on

pause(2)
%% Optimized trajectory
% 
% %thb0_list = linspace(pi/2 - 0.25, pi/2 + 0.25, 8);
% thb0_list = pi/4;
% 
% for i = 1:length(thb0_list)
% 
% soln = optimizeLandingTraj(thb0_list(i), jumper);
% 
% T = soln.grid.time;
% X = soln.grid.state;
% U = soln.grid.control;
% end
% %%
% figure(2); %hold on;
% % plot(T, X(1,:))
% % plot(T, X(2,:))
% draw_jumper(T, X', jumper, 2);
% %plot(T, U);
% % 
jumper.umax = umax;
jumper.dphi0 = dphi0; % initial wheel velocity (8)

% Generating optimal traj for tracking using fmincon (got it workign)
ref_traj = generate_tracking_lqr(jumper, jump_theta, K, 'fmincon');

X0 = [jump_theta; 0];
tspan = [0, 5];
lands = @(t,X) landing_event(t, X, jumper);
opts = odeset('Events', lands);
controller = @tracking_lqr_controller;
i = 3;

% Pass ref_traj as X_ref parameter
[tsim, xsim, te, xe, ie] = ode45(@(t, X) jumper_dynamics(t, X, controller, K, ref_traj, [], jumper), tspan, X0, opts);

% Calculate control and energy
uout = 0*tsim;
for n=1:length(tsim)
    [~,uout(n)] = jumper_dynamics(tsim(n),xsim(n,:)', controller, K, ref_traj, [], jumper);
end

% calc wheel speed
Jt = jumper.Jb + jumper.mw*jumper.Lbw^2;
H0 = E_wheel + Jt * xsim(1,2);
dphi_w = (H0 - Jt*xsim(:,2))/jumper.Jw;

% total effort
w_rel = dphi_w - xsim(:, 2);
E_total = cumtrapz(tsim, uout.*w_rel);
effort = cumtrapz(tsim, abs(uout.*w_rel));

TotE(i) = effort(end);
dphi_w_track = dphi_w;
xend(i) = xsim(end, 1);
yend(i) = xsim(end, 2);



%% Animate
figure('Name', 'Tracking LQR Controller', 'Position', fig_pos);%[100 100 1200 800]);
subplot(3, 2, [1,2]);
draw_jumper(tsim, xsim, jumper, 1, dphi_w);

%% Plot
subplot(3, 2, 3); hold on;
yyaxis left; hold on
plot(tsim, xsim(:,1)*180/pi, "-c", "LineWidth", 2)
plot(ref_traj.time, ref_traj.theta*180/pi, "--b")

ylabel('Angle (deg)')
ylim([0.9*min(180/pi*xsim(:,1)), max(180/pi*xsim(:,1))*1.1])
grid on

% Body angular velocity
yyaxis right; hold on
plot(tsim, xsim(:,2), "-r", "LineWidth", 2)
plot(ref_traj.time, ref_traj.dtheta, "--g")

ylabel('Angular Velocity (rad/s)')
title('Body States')
legend(["Body Angle"; "Reference Angle"; "Angular Velocity"; "Reference Velocity"], "Location", "northwest")
ylim([min(xsim(:,2)) - 0.2, max(xsim(:,2))*1.1])
grid on

% Control torque
subplot(3,2,4)
plot(tsim, uout)
ylabel('Torque (N*m)')
title('Input Torque')
grid on

subplot(3,2,5)
plot(tsim, dphi_w)
ylabel('Speed (rad/s)')
xlabel('Time (s)')
title('Calculated Flywheel Speed')
grid on

subplot(3,2,6); hold on;
Ewheel = 0.5 * jumper.Jw * dphi_w.^2;
w_rel = dphi_w - xsim(:, 2); % relative angular velocity of wheel
E_total = cumtrapz(tsim, uout.*w_rel);
effort = cumtrapz(tsim, abs(uout.*w_rel));
plot(tsim, E_total)
plot(tsim, effort)
ylabel('Energy (J)')
xlabel('Time (s)')
title('Flywheel Kinetic Energy')
legend(["Energy used"; "Total Energy used"], "location", "northwest")
grid on
pause(2)

%% Energy Comparison Plots (Non-Regenerative)

% Calculate initial and final wheel energies for each controller
E_wheel_initial = 0.5 * jumper.Jw * jumper.bdphi0^2;

% LQR Controller (i=1)
E_wheel_final_lqr = 0.5 * jumper.Jw * dphi_w_lqr(end)^2;
%wheel_energy_consumed_lqr = E_wheel_initial - E_wheel_final_lqr;

% Braking Controller (i=2)  
E_wheel_final_brake = 0.5 * jumper.Jw * dphi_w_brake(end)^2;  % Should be ~0
wheel_energy_consumed_brake = E_wheel_initial;% - E_wheel_final_brake;

% Tracking LQR Controller (i=3)
E_wheel_final_track = 0.5 * jumper.Jw * dphi_w_track(end)^2;
wheel_energy_consumed_track = E_wheel_initial - E_wheel_final_track;

% Total energy accounting (non-regenerative):
% - For LQR & Tracking LQR: Motor work is ALREADY in TotE, just add wheel energy lost
% - For Braking: The brief motor torque is negligible, main cost is wheel energy destroyed
TotE_compare = zeros(1,3);
TotE_compare(1) = TotE(1);% + wheel_energy_consumed_lqr;      % Motor + wheel loss
TotE_compare(2) = abs(wheel_energy_consumed_brake);              % Just wheel loss (motor work is tiny)
TotE_compare(3) = TotE(3);% + wheel_energy_consumed_track;    % Motor + wheel loss

%% Figure 1: Bar Chart Comparison
figure('Name', 'Energy Comparison (Non-Regenerative)', 'Position', [100 100 800 600]);
bar(TotE_compare)
ylabel("Total Energy Used In-air (J)");
set(gca, 'XTickLabel', {'LQR', 'Braking', 'Tracking LQR'});
title(sprintf("Energy Comparison (Non-Regen) - m_w = %.2f kg", jumper.mw))
ylim([0 max(TotE_compare)*1.2])
grid on

% Add text labels on bars
for i = 1:length(TotE_compare)
    text(i, TotE_compare(i) + max(TotE_compare)*0.05, ...
        sprintf('%.4f J', TotE_compare(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10);
end

%% Figure 2: Energy Breakdown (motor vs wheel)
figure('Name', 'Energy Breakdown (Non-Regenerative)', 'Position', [100 100 900 600]);

% For stacked bar chart:
% LQR: Motor work + wheel energy consumed
% Braking: No motor work (negligible), only wheel energy
% Tracking: Motor work + wheel energy consumed

motor_energy = [TotE(1), 0, TotE(3)];  % Braking has negligible motor work
%wheel_energy_consumed = [wheel_energy_consumed_lqr, wheel_energy_consumed_brake, wheel_energy_consumed_track];
wheel_energy_consumed = [0, abs(wheel_energy_consumed_brake),0];


bar_data = [motor_energy; wheel_energy_consumed]';

bar(bar_data, 'stacked')
ylabel("Energy (J)");
xlabel("Controller");
set(gca, 'XTickLabel', {'LQR', 'Braking', 'Tracking LQR'});
title(sprintf("Energy Breakdown (Non-Regen) - m_w = %.2f kg", jumper.mw))
legend({'Motor Work (Non-Regen)', 'Wheel Energy Consumed'}, 'Location', 'northwest')
ylim([0 max(TotE_compare)*1.2])
grid on

% Add text labels on bars
for i = 1:length(TotE_compare)
    text(i, (motor_energy(i) + wheel_energy_consumed(i))+0.02, ...
        sprintf('%.4f J', motor_energy(i) + wheel_energy_consumed(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10);
end
%% Figure 3: Landing Accuracy vs Energy Trade-off
figure('Name', 'Accuracy vs Energy', 'Position', [100 100 800 600]);

% Calculate landing errors
angle_error = abs([xend(1), xend(2), xend(3)] - jumper.final_theta) * 180/pi;
velocity_error = abs([yend(1), yend(2), yend(3)]);

scatter(TotE_compare, angle_error, 50, 'filled')
xlabel("Total Energy Used (J)");
ylabel("Final Angle Error (deg)");
title("Landing Accuracy vs Energy Trade-off")
grid on
%axis([0 0.55 -0.08 3])

% Add controller labels
text(TotE_compare(1), angle_error(1), '  LQR', 'FontSize', 12, 'VerticalAlignment', 'middle');
text(TotE_compare(2), angle_error(2), '  Braking', 'FontSize', 12, 'VerticalAlignment', 'middle');
text(TotE_compare(3), angle_error(3), '  Tracking LQR', 'FontSize', 12, 'VerticalAlignment', 'middle');


%% 
figure(6)


%%

function u = no_control(t, X, K, X_ref, U_ref, p)
% for testing with no control
    u = 0;
end

% checks if touched ground, ends ode45
function [value, isterminal, direction] = landing_event(t, X, p)
    % current height from parabolic motion
    y = p.y0 + p.vy0*t - 0.5*p.g*t^2;
    vy = p.vy0 - p.g*t;
    th_anim = pi/2 - X(1);

    y_bottom = y - p.Lb*abs(cos(th_anim));
    
    if(vy >= 0)
        value = 1;
    else
        % hits ground
        value = y_bottom;% + 0.0001;
    end
    isterminal = 1;  % Stops simulation
    direction = -1;  % Only down through y=0
end

function status = captureControl(t, X, flag, controller, K, X_ref, jumper)
% Captures control output during ODE45 integration
    global uout_log tsim_log
    
    if isempty(flag)
        % ODE45 is taking a step - log the control at these points
        for i = 1:length(t)
            [~, u] = jumper_dynamics(t(i), X(:,i), controller, K, X_ref, [], jumper);
            uout_log(end+1) = u;
            tsim_log(end+1) = t(i);
        end
    end
    
    status = 0;  % Continue integration
end