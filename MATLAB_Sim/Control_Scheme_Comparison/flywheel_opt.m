%code is to find the optimal radius needed 
%optimal is the minimum energy cost, both in terms of body weight and how
%it affects jump height as well as energy cost to get the correct momentum
%change with the flywheel.
clear 
close all
clc



%r ranges from values of 0-25 for the radius 
r = linspace(0.01,.03,1000); 

%% PART 1: Energy motor has to impart for the jumper to reach a height h as
%mass increases
rho=2700;% kg/m^3 aluminum
t=.006; %quarter inch thickness 
L=.00235;  %kength of the jumper


%lets say that we need 1J of energy when m=1. 2J when m=2

m = r.^2 .* t .* pi .* rho;
%1w motor is 5 g (.005). rise/run on a power vs mass plot
%E_mass=200*m;

%^ THIS WAS WRONG
%we need to find the elastic energy input needed as mass increases and get
%the sam eheight

%say we want to jump 5 meter
%say the mass is evenly split between the foot and the body
%h=(1/2g)kx^2*mb/ms^2

h=3; %jump height. meters
g=9.81;
mb=m;
ms=2*m;
%E_mass=h.*g.*(2*m).^2/m;
E_mass=4*h*g*m;

%ih paper, can do a bunch of these blue line familes!!!
%% PART 2: Energy cost to get the correct momentum in the flywheel

%this just energy to spin flywheel up. 
%series of red curves for different L. As just higher, can get away with
%lower than necessary L. 
%have the height connected...L is the maount of momentum need to change.
%NO. time addects the motor power. Time does not affect L. it is how big
%robot is and how many degrees need to rotate. 

%this assumes the robot is static at top and not rotating. this is the
%worst case scenerio. 


E_mom = 0.5 .* L^2 ./ (r.^4 .* t .* pi .* rho);

E_tot= E_mom+E_mass;
[min_val, min_idx] = min(E_tot);
min_r = r(min_idx);
fprintf("Minimized energy cost is at r = %1.3f meters\n", min_r)
%% plot E_mass vs r and E_mom vs r on same plot

figure;
plot(r, E_mass, 'b', 'LineWidth', 2); hold on;
plot(r, E_mom, 'r', 'LineWidth', 2);
plot(r, E_tot, 'k', 'LineWidth', 2);
xline(min_r, "--")
xlabel('Flywheel radius (m)');
ylabel('Energy Cost');
legend('E_{mass}', 'E_{mom}', 'E_{total}', 'r_{min}', "Location", "northwest");
title('E_{mass} and E_{mom} vs r');
grid on;
ylim([0 4]);
xlim([0 .03]);

%% plot  E_tot vs r

figure;
plot(r, E_tot, 'k', 'LineWidth', 2);
xlabel('Flywheel Radius (m)');
ylabel('Total Energy Cost');
title('E_{tot} vs r');
xline(min_r, "--")
grid on;

ylim([0 10]);
xlim([0 .03]);

figure;
plot(r, m)
xlabel('r [m]');
ylabel('Mass [kg]');
title('E_{tot} vs m');
grid on


xlim([0 .03])