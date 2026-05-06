%% Quaternion Cube + Reaction Wheel Simulation (PD Attitude Control)
clear; clc; close all;

recording = 0;

if recording == 1
    v = VideoWriter('pd_controlled_cube.mp4', 'MPEG-4');
    v.FrameRate = 30;
    open(v)
end

fig = figure;
set(fig, 'Units', 'pixels', ...
         'Position', [100 100 480 720], ...
         'MenuBar', 'none', ...
         'ToolBar', 'none');

% ===== Cube geometry =====
l = 0.5;

V = (l/2) * [
    -1 -1 -1
     1 -1 -1
     1  1 -1
    -1  1 -1
    -1 -1  1
     1 -1  1
     1  1  1
    -1  1  1
];

F = [
    1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
];

% ===== Simulation parameters =====
z0 = 0.1;
v0 = 15;
dt = 1/100;
g = 9.81;
theta = 45;
vx0 = v0*cosd(theta);
vz0 = v0*sind(theta);

tc = 0;
landed = false;

% ===== Inertia =====
mr = 0.1;
Rr = 0.1;
mc = 0.2;
lc = 0.5;

Ic = mc*lc^2/6;     % cube inertia
Ir = mr*Rr^2;       % wheel inertia

% Landing

a = -0.5*9.81;
b = vx0;
c = z0;
t_land = (-b - sqrt(b^2-4*a*c))/(2*a);

x_land = vx0*t_land;

t_top = vx0/g;
z_top = z0 + vz0*t_top - .5*g*t_top^2;

% ===== Initial angular velocities =====
wXYZ = [0; 0; 0];
wr   = [0; 0; 0];

wc0 = wXYZ;
wr0 = wr;

% ===== Quaternion =====
q = [1 0 0 0]';

% ===== Desired orientation (flat face down) =====

qd = [1 0 0 0]; % identity
%q_d = [0 1 0 0];
%qd = [0.7071; -0.7071; 0; 0];
%qd = [0.7071; 0; -0.7071; 0];

%    desired orientation - track velocity vector





% ===== Control gains =====
Kp = 1;
Kd = .1;

% ===== Position =====.
xc = 0;
yc = 0;
zc = z0;

% ===== Data storage =====
t_flight = 2*v0/g;
N = ceil(t_flight / dt);

t_hist  = zeros(N,1);
w_hist  = zeros(N,3);
wr_hist = zeros(N,3);

i = 1;

qe_all = zeros(N, 4);

% ===== MAIN LOOP =====
while ~landed
    
    % ---- Velocity and Translation ----
    vz = vz0 - g*tc;
    vx = vx0;
    %vvec = atan2d(vz, vx)


    zc = z0 + vz0*tc - 0.5*g*tc^2;
    xc = vx0*tc;

    % Find velocity vector and generate required rotation from body

    vvec = [vx; 0; vz] / norm([vx; 0; vz])';
    body_axis = [0; 0; -1];

    c  = dot(body_axis, vvec);
    ax = cross(body_axis, vvec);

    qd_col = [1 + c; ax];
    qd = (qd_col / norm(qd_col))';
    
    % ---- Quaternion error ----
    q_conj = [q(1); -q(2:4)];
    qe = quatmultiply(q_conj, qd);   % both column vectors

    
    % if qe(1) < 0
    %     qe = -qe;
    % end
    
    e_rot = sign(qe(1)) * qe(2:4);
    
    % ---- PD Control ----
    tau = -Kp * e_rot + Kd * wXYZ;
    %tau(3,1)=0;
    
    % ---- Dynamics ----
    wc = wXYZ;
    cross_term = cross(wc, wr);
    
    wc_dot = -(Ir/Ic)*cross_term - tau/Ic;
    wr_dot = tau / Ir;
    
    % ---- Integrate ----
    wXYZ = wXYZ + wc_dot * dt;
    wr   = wr   + wr_dot * dt;
    
    % ---- Quaternion Integration ----
    W = [  0        -wXYZ(1) -wXYZ(2) -wXYZ(3);
           wXYZ(1)   0         wXYZ(3) -wXYZ(2);
           wXYZ(2)  -wXYZ(3)   0        wXYZ(1);
           wXYZ(3)   wXYZ(2)  -wXYZ(1)  0 ];
    
    qdot = 0.5 * W * q;
    q = q + qdot * dt;
    q = q / norm(q);
    
    % ---- Rotation matrix ----
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    
    Rm = [1 - 2*(q2^2 + q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
          2*(q1*q2 + q0*q3),   1 - 2*(q1^2 + q3^2),   2*(q2*q3 - q0*q1);
          2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
    
    % ---- Rotate cube ----
    V_rot = (Rm * V')';
    V_world = V_rot + [xc yc zc];
    
    % ---- Plot ----
    clf
    hold on
    
    faceColors = [
        1 0 0
        0 1 0
        0 0 1
        1 1 0
        1 0 1
        0 1 1
    ];
    
    patch('Vertices', V_world, ...
          'Faces', F, ...
          'FaceVertexCData', faceColors, ...
          'FaceColor', 'flat', ...
          'EdgeColor', 'k', ...
          'LineWidth', .5);
    
    axis equal
    camproj('orthographic')
    axis([-1.5 x_land+1.5 -1.5 1.5 -1.5 z_top+1.5])
    grid on
    xlabel('X'); ylabel('Y'); zlabel('Z')
    
    title('PD Attitude Controlled Cube')
    subtitle(sprintf(['Kp=%.1f  Kd=%.1f\n t=%.2f\n', ...
        '\\omega_c=[%.2f %.2f %.2f]\n', ...
        '\\omega_r=[%.2f %.2f %.2f]'], ...
        Kp, Kd, tc, ...
        wXYZ(1), wXYZ(2), wXYZ(3), ...
        wr(1), wr(2), wr(3)));
    
    view(3)
    pause(dt/5);
    hold off
    
    if recording == 1
        frame = getframe(gcf);
        writeVideo(v,frame)
    end
    
    % ---- Store data ----
    t_hist(i)  = tc;
    w_hist(i,:)  = wXYZ';
    wr_hist(i,:) = wr';
    qe_all(i, :) = qe';
    
    i = i + 1;
    
    % ---- Time update (FIXED: only once) ----
    tc = tc + dt;
    
    if zc < 0
        landed = true;
    end
end

if recording == 1
    close(v)
end

% ===== Trim =====
t_hist  = t_hist(1:i-1);
w_hist  = w_hist(1:i-1,:);
wr_hist = wr_hist(1:i-1,:);
qe_all = qe_all(1:i-1,:);

% ===== Plot results =====
figure;

subplot(2,1,1)
plot(t_hist, w_hist, 'LineWidth', 1.5);
grid on
xlabel('Time (s)')
ylabel('\omega_c')
title('Cube Angular Velocity')
legend('x','y','z')

subplot(2,1,2)
plot(t_hist, wr_hist, '--', 'LineWidth', 1.5);
grid on
xlabel('Time (s)')
ylabel('\omega_r')
title('Reaction Wheel Speeds')
legend('r1','r2','r3')

% ===== Quaternion multiply =====
function q = quatmultiply(q1, q2)
    % Ensure column vectors
    q1 = q1(:);
    q2 = q2(:);

    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);

    q = [
        w1*w2 - x1*x2 - y1*y2 - z1*z2;
        w1*x2 + x1*w2 + y1*z2 - z1*y2;
        w1*y2 - x1*z2 + y1*w2 + z1*x2;
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ];
end