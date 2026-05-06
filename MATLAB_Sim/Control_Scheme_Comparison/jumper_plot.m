p = jumper;

% Body shape
    wb = p.wb;
    Lb = p.Lb;
%     yu = [-.5 0 .5 .5 0 -.5 -.5]*wb;
%     xu = [-.4 -.5 -.4 .4 .5 .4 -.4]*Lb;
    xu = [-1 0 1 1 0 -1 -1]*wb;
    yu = [-1 -1 -1 1 1 1 -1]*Lb;
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

    figure
    hold on
    plot(xu, yu)
    plot(xw, yw)
    axis equal