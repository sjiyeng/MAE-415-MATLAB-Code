clearvars
d_minor = 12.9E-3; % https://amesweb.info/screws/metric-trapezoidal-thread-calculator.aspx
d_major = 16E-3; %nominal diameter
L = 600E-3;        % ballscrew length (m)
E = 193E9;         % 304 stainless elastic modulus (Pa)
Sy = 215E6;        % 304 stainless yield strength (Pa) - adjust if using cold-worked
K = 1.0;           % pinned at both ends (Theoretical K = 1.0)
c = 4000;          % N*s/m, damping constant
v = 600E-3;        % m/s, max velocity
N = 2;             % number of ballscrews
P = 2000;          % Wattage of power supply

% Geometric & Slenderness Calculations
A = (pi * d_minor^2) / 4;% Cross-sectional area (m^2)
I = (pi * d_minor^4) / 64;% Area moment of inertia (m^4)
r_gyr = sqrt(I / A);% Radius of gyration (d_minor / 4)
Le = K * L;% Effective length (m)

SR = Le / r_gyr; % Actual Slenderness Ratio (Le / r)
SR_crit = sqrt((2 * pi^2 * E) / Sy); % Critical Slenderness Ratio 

% Buckling Calculation Selection (Euler vs. Johnson)
if SR >= SR_crit
    buckling_method = "Euler";
    Pcr = (pi^2 * E * I) / (Le^2);
else
    buckling_method = "Johnson";
    Pcr = A * (Sy - (1 / E) * ((Sy * SR) / (2 * pi))^2);
end

% Force & Power Calculations
Fshock = (c * v) / N;
c_max = P / v^2;

% Output


fprintf("Buckling Calculations (%s Method Used):\n", buckling_method);
fprintf("c = %.3f W and v = %.3f m/s\n", c, v);
fprintf("Ballscrew Minor Diameter = %.3f mm\n",d_minor * 1000);
fprintf("Ballscrew Major Diameter = %.3f mm\n",d_major * 1000);
fprintf("Slenderness Ratio (Le/r) = %.3f (Critical Ratio = %.3f)\n", SR, SR_crit);
fprintf("Critical Buckling Force = %.3f N\n", Pcr);
fprintf("Shock Force per Screw = %.3f N\n", Fshock);
fprintf("Safety Factor = %.3f\n", Pcr / Fshock);