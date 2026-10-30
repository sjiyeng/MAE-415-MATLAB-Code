%% constants
P = 1.0E3; %W
n_avg = 0.95; %efficiency
Troom = 22; %c
Tboard_max = 85; %rated maximum board temp

d_fan = 60E-3; %m
Vdot_fan = 65.74 / 2119; %m^3/s, converted from CFM

v = 1.48E-5; %kinematic viscocity, m^2/s
L = 46E-3; %width of board
Pr = 0.71; %prantdl number of air
k = .024; %w/m*k, thermal conductivity of air
% link to fan: https://www.amazon.com/SilverStone-Technology-Performance-Industrial-SST-FHS60X/dp/B0FC5K4SL8

%Calculations
A_fan = .25 * pi * d_fan^2; %m^2
V_air = Vdot_fan / A_fan; %m/s
%reynolds number
Re = (V_air*L) / v;

if Re > 5E5 
    fprintf("Flow is mixed/turbulent\n");
    Nu_L = (.037 * Re^0.8 - 871) * Pr^.333;
else
    fprintf("Flow is laminar\n");
    Nu_L = 0.664 * Re^0.5 * Pr^.333;
end

syms Tboard
h_avg = (Nu_L * k) / L;
a_eff = L^2 * 15; %heatsink with 15x the surface area
q_out = h_avg * (Tboard - Troom) * a_eff;
q_in = P * (1-n_avg);
qnet = q_in - q_out;

Tboard = vpasolve(qnet == 0, Tboard);

fprintf("Board Temperature = %.3f degrees C \n",Tboard);
if Tboard < Tboard_max
    fprintf("Board Temperature less than max allowable, cooling is sufficient\n");
else
    fprintf("Board Temperature more than max allowable, cooling is insufficient\n");
end
