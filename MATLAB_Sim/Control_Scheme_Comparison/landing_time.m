function [t_land, x_land] = landing_time(p)

g = p.g; vx0 = p.vx0; vy0 = p.vy0; y0 = p.y0;
% solving 0.5*g*t^2 - vy0*t - y0 = 0

dsc = vy0^2 + 4*0.5*g*y0;

if(dsc < 0)
    error("Impossible system");
end

t_land1 = (vy0 + sqrt(dsc))/g;
t_land2 = (vy0 - sqrt(dsc))/g;

if(t_land1 > 0 && t_land2 > 0)
    t_land = max(t_land1, t_land2);
elseif(t_land1 > 0)
    t_land = t_land1;
elseif(t_land2 > 0)
    t_land = t_land2;
else
    error("No Landing");
end

x_land = vx0*t_land;

end