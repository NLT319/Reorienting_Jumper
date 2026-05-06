% function draw_jumper(tout, xout, p, fig)
% 
% % Adapted for reduced 2-state system
% % xout = [thb; dthb] (N x 2 matrix)
% % Wheel angle calculated from angular momentum conservation
% 
%     dt = 0.02;
%     
%     % Body shape
%     wb = p.wb;
%     Lb = p.Lb;
%     yu = [-.5 0 .5 .5 0 -.5 -.5]*wb;
%     xu = [-.4 -.5 -.4 .4 .5 .4 -.4]*Lb;
%     au = atan2(yu,xu);
%     ru = (xu.^2 + yu.^2).^.5;
%     
%     % Wheel
%     Rw = p.Rw;
%     Lbw = p.Lbw;
%     av = 2*pi*[0:.01:1];
%     xw = Rw*cos(av);
%     yw = Rw*sin(av);
%     Lw = 1.2*Rw;
%     hw = 0.3*Rw;
%     xw2 = [-1 1 1 -1 -1]*Lw*0.5;
%     yw2 = [-1 -1 1 1 -1]*hw*0.5;
%     aw2 = atan2(yw2,xw2);
%     rw2 = (xw2.^2 + yw2.^2).^.5;
%     
%     % Colors
%     cw = [.6 .9 1];
%     cw2 = [.3 .3 .3];
%     cb = [.2 .8 .1];
%     
%     % Interpolation
%     tu = [0:dt:max(tout), tout(end)];
%     thb = interp1(tout, xout(:,1), tu);   % Body angle
%     dthb = interp1(tout, xout(:,2), tu);  % Body angular velocity
%     
%     % Calculate wheel angle from angular momentum conservation
%     Jt = p.Jb + p.mw*p.Lbw^2;
%     H0 = p.Jw * p.dphi0 + Jt * xout(1,2); % initial angular momentum
%     
%     % Wheel angular velocity from conservation
%     dphi_w = (H0 - Jt * dthb) / p.Jw;
%     
%     % Integrate to get wheel angle
%     phib = cumtrapz(tu, dphi_w);
%     
%     % Calculate COM trajectory from jump physics
%     x_com = p.vx0 * tu;
%     y_com = p.y0 + p.vy0 * tu - 0.5 * p.g * tu.^2;
%     
%     Anim = subplot(3,2,[1,2]);
%     hold(Anim,'on'); 
%     axis(Anim,'equal');
%     axis(Anim, [-0.5 max(x_com)*1.1 -0.2 max(y_com)*1.5]);
%     
%     for n = 1:length(tu)
%         cla(Anim);
%         hold on
%         axis image
%         axis([-.5 max(x_com)*1.1 -.2 max(y_com)*1.5]);
%         
%         % Current positions
%         x0 = x_com(n);
%         y0 = y_com(n);
%         th = thb(n);
%         phi = phib(n);
%         
%         % Body
%         x_body = x0 + ru .* cos(au + th);
%         y_body = y0 + ru .* sin(au + th);
%         
%         % Wheel rim
%         x_wheel = xw + x0 + Lbw * cos(th);
%         y_wheel = yw + y0 + Lbw * sin(th);
%         
%         % Wheel indicator (shows wheel rotation)
%         x_indicator = x0 + rw2 .* cos(aw2 + phi) + Lbw * cos(th);
%         y_indicator = y0 + rw2 .* sin(aw2 + phi) + Lbw * sin(th);
%         
%         % Draw everything
%         patch(x_body, y_body, 'g', 'FaceColor', cb, 'FaceAlpha', .1);
%         patch(x_wheel, y_wheel, 'w', 'FaceColor', cw, 'FaceAlpha', .1);
%         patch(x_indicator, y_indicator, 'k', 'FaceColor', cw2, 'FaceAlpha', 1);
%         
%         % Ground
%         plot([-10 200], [0 0], 'k-', 'LineWidth', 2);
%         
%         % Trajectory
%         plot(x_com(1:n), y_com(1:n), 'b--', 'LineWidth', 1);
%         
%         title(sprintf('t = %.2f s, theta = %.1f°, omega_{wheel} = %.1f rad/s', ...
%             tu(n), rad2deg(th), dphi_w(n)));
%         
%         drawnow;
%         pause(0.005);
%     end
% pause(1);
% axis([-.5 max(x_com)*1.1 -.2 max(y_com)*1.5]);
% 
% end

function draw_jumper(tout, xout, p, fig, dphi_w_in)
% Adapted for reduced 2-state system
% xout = [thb; dthb] (N x 2 matrix)
% Wheel angle calculated from angular momentum conservation
    dt = 0.02;
% Body shape
    wb = p.wb;
    Lb = p.Lb;
%     yu = [-.5 0 .5 .5 0 -.5 -.5]*wb;
%     xu = [-.4 -.5 -.4 .4 .5 .4 -.4]*Lb;
    yu = [-1 0 1 1 0 -1 -1]*wb;
    xu = [-1 -1 -1 1 1 1 -1]*Lb;
    au = atan2(yu,xu);
    ru = (xu.^2 + yu.^2).^.5;
% Wheel
    Rw = p.Rw;
    Lbw = p.Lbw;
    av = 2*pi*[0:.01:1];
    xw = Rw*cos(av);
    yw = Rw*sin(av);
    Lw = 1.2*Rw;
    hw = 0.3*Rw;
    xw2 = [-1 1 1 -1 -1]*Lw*0.5;
    yw2 = [-1 -1 1 1 -1]*hw*0.5;
    aw2 = atan2(yw2,xw2);
    rw2 = (xw2.^2 + yw2.^2).^.5;
% Colors
    cw = [.6 .9 1];
    cw2 = [.3 .3 .3];
    cb = [.2 .8 .1];
% Interpolation
    tu = [0:dt:max(tout), tout(end)];
    thb = interp1(tout, xout(:,1), tu);
    dthb = interp1(tout, xout(:,2), tu);
% Calculate wheel angle from angular momentum conservation
    Jt = p.Jb + p.mw*p.Lbw^2;
    H0 = p.Jw * p.dphi0 + Jt * xout(1,2);
    %dphi_w = (H0 - Jt * dthb) / p.Jw;
    dphi_w = interp1(tout, dphi_w_in, tu);
    phib = cumtrapz(tu, dphi_w);
% Calculate COM trajectory from jump physics
    x_com = p.vx0 * tu;
    y_com = p.y0 + p.vy0 * tu - 0.5 * p.g * tu.^2;

    % --- View parameters ---
    zoom_pad   = max(p.Lb, p.Rw) * 1.5;   % padding around jumper during zoom
    full_xlim  = [min(x_com)-0.5,  max(x_com)*1.1];
    full_ylim  = [-0.2,             max(y_com)*1.5];
    zoom_frames = 30;                    % frames for zoom-out transition at end

    Anim = subplot(3,2,[1,2]);
    hold(Anim,'on');
    axis(Anim,'equal');


    for n = 1:length(tu)
        cla(Anim);
        hold on
        axis image

        % ---- ZOOMED view centred on jumper ----
        x0 = x_com(n);
        y0 = y_com(n);
        ax_zoom = [x0 - zoom_pad, x0 + zoom_pad, ...
                   max(-0.2, y0 - zoom_pad), y0 + zoom_pad];
        axis(ax_zoom);

        th  = thb(n);
        phi = phib(n);

        % Body
        x_body = x0 + ru .* cos(au + th);
        y_body = y0 + ru .* sin(au + th);
        % Wheel rim
        x_wheel = xw + x0 + Lbw * cos(th);
        y_wheel = yw + y0 + Lbw * sin(th);
        % Wheel indicator
        x_indicator = x0 + rw2 .* cos(aw2 + phi) + Lbw * cos(th);
        y_indicator = y0 + rw2 .* sin(aw2 + phi) + Lbw * sin(th);

        patch(x_body,      y_body,      'g', 'FaceColor', cb,  'FaceAlpha', .1);
        patch(x_wheel,     y_wheel,     'w', 'FaceColor', cw,  'FaceAlpha', .1);
        patch(x_indicator, y_indicator, 'k', 'FaceColor', cw2, 'FaceAlpha',  1);
        plot([-10 200], [0 0], 'k-', 'LineWidth', 2);
        plot(x_com(1:n),   y_com(1:n),  'b--', 'LineWidth', 1);

        title(sprintf('t = %.2f s, theta = %.1f°, omega_{wheel} = %.1f rad/s', ...
            tu(n), rad2deg(th), dphi_w(n)));
        drawnow;
        if(n == 1)
            pause(1);
        end

        pause(0.005);
    end

    % ---- ZOOM-OUT transition to full trajectory ----
    n = length(tu);
    x0 = x_com(n);  y0 = y_com(n);
    ax_start = [x0 - zoom_pad,    x0 + zoom_pad, ...
                max(-0.2, y0 - zoom_pad), y0 + zoom_pad];
    ax_end   = [full_xlim(1), full_xlim(2), full_ylim(1), full_ylim(2)];

    for k = 1:zoom_frames
        alpha = k / zoom_frames;               % 0 → 1
        alpha_smooth = alpha^2 * (3 - 2*alpha); % smoothstep easing
        ax_now = ax_start + alpha_smooth * (ax_end - ax_start);

        cla(Anim);
        hold on
        axis(ax_now);

        % Redraw final pose
        x_body      = x0 + ru .* cos(au + thb(n));
        y_body      = y0 + ru .* sin(au + thb(n));
        x_wheel     = xw + x0 + Lbw * cos(thb(n));
        y_wheel     = yw + y0 + Lbw * sin(thb(n));
        x_indicator = x0 + rw2 .* cos(aw2 + phib(n)) + Lbw * cos(thb(n));
        y_indicator = y0 + rw2 .* sin(aw2 + phib(n)) + Lbw * sin(thb(n));

        patch(x_body,      y_body,      'g', 'FaceColor', cb,  'FaceAlpha', .1);
        patch(x_wheel,     y_wheel,     'w', 'FaceColor', cw,  'FaceAlpha', .1);
        patch(x_indicator, y_indicator, 'k', 'FaceColor', cw2, 'FaceAlpha',  1);
        plot([-10 200], [0 0], 'k-', 'LineWidth', 2);
        plot(x_com, y_com, 'b--', 'LineWidth', 1);   % full trajectory

        title('Jump complete — full trajectory');
        drawnow;
        pause(0.02);
    end

    pause(1);
    axis([full_xlim full_ylim]);
end