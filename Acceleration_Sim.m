%% SHOCK DYNAMOMETER SIMULATION - DUAL EAGLEPOWER 8308 (KV90)
% =========================================================================
% North Carolina State University - Pack Motorsports Baja
% Department of Mechanical and Aerospace Engineering (MAE 415)
%
% ARCHITECTURE OVERVIEW:
%   Simulates a dual-ballscrew, dual-BLDC linear shock dynamometer driven
%   by two Eaglepower 8308 (KV90) outrunner motors and Moteus FOC drives
%   powered from a 48V, 2 kW DC power supply.
%
% TWO-DEADZONE TRAJECTORY STATE MACHINE:
%   - Set 1 (Outer Deadzone Limits): 3% (pct_lim1_low) & 97% (pct_lim1_high)
%     Defines the safe turnaround dwell positions with a 3% buffer at each
%     end of the physical shock absorber stroke (0% and 100%).
%   - Set 2 (Acceleration Cutoff Limits): 80% (pct_lim2_low) & 20% (pct_lim2_high)
%     Defines the maximum stroke fraction over which motoring torque is
%     applied during extension (up to 80%) and retraction (down to 20%).
%
% MOTION CYCLE SEQUENCE:
%   1. Dwell at 3% lower outer limit for t_dwell (0.20 s) to ensure v = 0.
%   2. Accelerate upward (+x, Extension) with stepped torque (+tau_curr)
%      until reaching 80% stroke OR until the dynamic worst-case braking
%      runway (d_stop_worst) reaches the 97% upper limit cushion.
%   3. Apply full active braking (-I_max * Kt) until carriage stops (v = 0).
%   4. Crawl gently and ramp velocity smoothly to 0 m/s at the 97% limit.
%   5. Dwell at 97% upper outer limit for t_dwell (0.20 s) to ensure v = 0.
%   6. Accelerate downward (-x, Retraction) with stepped torque (-tau_curr)
%      until reaching 20% stroke OR until the dynamic worst-case braking
%      runway reaches the 3% lower limit cushion.
%   7. Apply full active braking (+I_max * Kt) until carriage stops (v = 0).
%   8. Crawl gently and ramp velocity smoothly to 0 m/s at the 3% limit.
%   9. Increment tau_curr by tau_step and repeat until hardware saturation
%      (Current, 2 kW Supply Power, or SVPWM Bus Voltage limit) is reached.
% =========================================================================

clear; clc; close all;

%% 1. SYSTEM PARAMETERS
% --- Dynamometer Orientation ---
is_horizontal   = true;             % true = Horizontal rail (no gravity bias); false = Vertical
g_accel         = 9.81;             % Gravitational acceleration [m/s^2]
g_axial         = is_horizontal * 0 + (~is_horizontal) * g_accel; % Axial gravity component [m/s^2]

% --- Shock Damper & Test Travel (CHANGE THESE FOR TEST CASES) ---
c_damp          = 100;              % Shock viscous damping coefficient [N*s/m] (e.g., 100 or 4000)
shock_stroke_m  = 8.0 * 0.0254;     % Working stroke of the shock [m] (e.g., 2.0*0.0254 or 8.0*0.0254)
x_start_m       = 0.050;            % Starting datum offset on dynamometer rail [m] (50 mm offset)

% Physical hard-stop boundaries of the shock absorber [m]
x_bottom        = x_start_m;                    % 0% stroke datum (fully compressed) [m]
x_top           = x_start_m + shock_stroke_m;   % 100% stroke datum (fully extended) [m]

% --- Two Sets of Deadzones / Trajectory Limits ---
% Set 1: Outer turnaround limits (3% and 97% of stroke)
pct_lim1_low    = 0.03;                         % Lower outer turnaround limit (3% of stroke)
pct_lim1_high   = 1.00 - pct_lim1_low;          % Upper outer turnaround limit (97% of stroke)

% Set 2: Inner acceleration cutoff limits (80% for extension, 20% for retraction)
pct_lim2_low    = 0.80;                         % Extension acceleration cutoff (80% of stroke)
pct_lim2_high   = 0.20;                         % Retraction acceleration cutoff (20% of stroke)

% Absolute rail positions corresponding to Set 1 and Set 2 limits [m]
x_lim1_low      = x_bottom + pct_lim1_low  * shock_stroke_m; % Lower Set 1 position (3% mark) [m]
x_lim2_low      = x_bottom + pct_lim2_low  * shock_stroke_m; % Extension accel cutoff (80% mark) [m]
x_lim2_high     = x_bottom + pct_lim2_high * shock_stroke_m; % Retraction accel cutoff (20% mark) [m]
x_lim1_high     = x_bottom + pct_lim1_high * shock_stroke_m; % Upper Set 1 position (97% mark) [m]

% Dwell pause duration at Set 1 outer limits to guarantee zero initial velocity [s]
t_dwell         = 0.20;

% --- Mechanical Transmission & Reflected Inertia ---
p_lead          = 0.010;            % Ballscrew lead [m/rev] (10 mm linear travel per revolution)
eta_screw       = 0.85;             % Ballscrew mechanical efficiency (forward/back-driving)
m_carriage      = 1.5;              % Linear moving mass of carriage + shock mount tooling [kg]
J_motor         = 2.5e-4;           % Rotational inertia of a single 8308 motor rotor [kg*m^2]

% Ballscrew shaft rotational inertia (solid steel cylinder, 14 mm diameter, 400 mm length)
d_screw         = 0.014;            % Ballscrew nominal shaft diameter [m]
L_screw         = 0.40;             % Ballscrew total length [m]
rho_steel       = 7850;             % Carbon steel density [kg/m^3]
m_screw         = rho_steel * (pi * (d_screw/2)^2 * L_screw); % Mass of one ballscrew shaft [kg]
J_screw         = 0.5 * m_screw * (d_screw/2)^2;             % Polar moment of inertia per screw [kg*m^2]

% Effective linear mass (M_eff):
% Converts rotational kinetic energy of 2 motors + 2 ballscrews into equivalent linear mass
% via kinematic transmission ratio (2*pi / p_lead)^2 = 394,784 rad^2/m^2.
rad_per_m       = (2 * pi) / p_lead;                         % Rotational-to-linear ratio [rad/m] (~628.32 rad/m)
M_eff           = m_carriage + 2 * (J_motor + J_screw) * (rad_per_m^2); % Total effective mass [kg] (~208.2 kg)

% --- Motor & Electrical (Dual Eaglepower 8308 KV90, Moteus FOC Controllers) ---
KV              = 90;                   % Motor velocity constant [RPM/V]
Kt              = 60 / (2 * pi * KV);   % Quadrature torque constant [N*m/A] (~0.1061 N*m/A)
R_phase         = 0.092;                % Datasheet line-to-line winding resistance (R_LL) [Ohms]
R_eff           = 0.75 * R_phase;       % 3-phase sinusoidal FOC effective resistance (3/4 * R_LL) [Ohms]
V_bus           = 48.0;                 % DC power supply bus voltage [V]
eta_svpwm       = 0.95;                 % Moteus SVPWM inverter max modulation duty cycle limit
V_eff           = eta_svpwm * V_bus;    % Effective maximum usable voltage ceiling [V] (45.6 V)

% --- Hardware Current, Power, & Friction Constraints ---
I_max           = 40.0;                 % Moteus peak phase current limit per motor [A]
P_sys_max       = 2000.0;               % Total 48V DC power supply capacity [W] (2 kW)
P_mot_max       = P_sys_max / 2.0;      % Maximum supply power allocation per motor [W] (1000 W)

% Maximum available braking force from both motors:
% During active deceleration, ballscrew thread friction opposes motion and aids braking (/ eta_screw).
F_max_brake     = 2 * (Kt * I_max * rad_per_m / eta_screw);
F_nut_drag      = 15.0;                 % Constant ballscrew nut preload + linear rail seal drag [N]

%% 2. TEST PROFILE & SOLVER CONFIGURATION
tau_start       = 0.30;       % Initial cycle test torque command per motor [N*m]
tau_step        = 0.35;       % Torque step increment added after each full cycle [N*m]
dt              = 0.0005;     % Numerical integration time step [s] (0.5 ms / 2 kHz)
t_max           = 60.0;       % Maximum simulation fallback timeout [s]

%% 3. SIMULATION INITIALIZATION
N = round(t_max / dt);
t_vec     = zeros(N, 1);      % Time vector [s]
x_vec     = zeros(N, 1);      % Absolute carriage position vector [m]
v_vec     = zeros(N, 1);      % Carriage linear velocity vector [m/s]
a_vec     = zeros(N, 1);      % Carriage linear acceleration vector [m/s^2]
I_vec     = zeros(N, 1);      % Motor quadrature phase current (I_q) per motor [A]
tau_vec   = zeros(N, 1);      % Delivered electromagnetic torque per motor [N*m]
P_sys_vec = zeros(N, 1);      % Net electrical power (+Supply Draw / -Regen Braking) [W]
state_vec = zeros(N, 1);      % Active state machine mode index

% State Machine Mode Definitions:
%   1: DWELL_AT_LIM1_LOW  - Hold stationary at 3% lower limit (pct_lim1_low) for t_dwell
%   2: EXTEND_ACCEL       - Accelerate upward (+x) with +tau_curr until 80% limit (pct_lim2_low) or braking runway
%   3: EXTEND_DECEL       - Full active braking (-I_max * Kt) until carriage comes to rest (v = 0)
%   4: JOG_TO_LIM1_HIGH   - Crawl upward at v_crawl and ramp smoothly to 0 m/s at 97% upper limit (pct_lim1_high)
%   5: DWELL_AT_LIM1_HIGH - Hold stationary at 97% upper limit (pct_lim1_high) for t_dwell
%   6: RETRACT_ACCEL      - Accelerate downward (-x) with -tau_curr until 20% limit (pct_lim2_high) or braking runway
%   7: RETRACT_DECEL      - Full active braking (+I_max * Kt) until carriage comes to rest (v = 0)
%   8: JOG_TO_LIM1_LOW    - Crawl downward at -v_crawl and ramp smoothly to 0 m/s at 3% lower limit (pct_lim1_low)

state_mode      = 1;          % Begin simulation in State 1 (dwell at 3% lower limit)
dwell_timer     = 0.0;        % Elapsed dwell timer [s]
x               = x_lim1_low; % Initialize carriage position at exact 3% mark [m]
v               = 0.0;        % Initialize carriage velocity at 0 [m/s]
tau_curr        = tau_start;  % Initialize test torque command [N*m]
cycle_count     = 0;          % Counter for completed extension + retraction cycles
hit_limit       = false;      % Flag indicating hardware saturation limit reached
last_idx        = N;          % Index of final recorded time step

% Closed-loop slow crawl controller parameters (used in States 4 and 8)
v_crawl         = 0.040;      % Maximum gentle crawl speed [m/s] (40 mm/s)
k_pos_crawl     = 15.0;       % Proportional position-approach gain [1/s] (ramps v_ref -> 0 over final ~2.7 mm)
k_vel_track     = 8000;       % Proportional velocity tracking gain [N/(m/s)]

% Hardware bottleneck diagnostic tracking
v_at_first_limit     = NaN;
limit_type_str       = 'None';
limit_captured       = false;

%% 4. NUMERICAL INTEGRATION LOOP (Semi-Implicit Euler at 2 kHz)
for k = 1:N
    t = (k - 1) * dt;
    
    % Model-free worst-case (c = 0) kinematic stopping distance from current velocity v:
    % Guarantees the carriage always initiates braking early enough to stop before the
    % 3% and 97% outer limits, even when testing an unknown or very low-damping shock.
    d_stop_worst     = (M_eff * v^2) / (2 * (F_max_brake + F_nut_drag));
    x_safety_cushion = 0.0010; % 1.0 mm safety margin inside the 3% and 97% limits [m]
    
    % --- Trajectory State Machine ---
    switch state_mode
        case 1 % DWELL_AT_LIM1_LOW: Stationary pause at 3% lower outer limit
            tau_cmd     = 0.0;
            v           = 0.0;
            x           = x_lim1_low;
            dwell_timer = dwell_timer + dt;
            if dwell_timer >= t_dwell
                dwell_timer = 0.0;
                state_mode  = 2; % Transition to EXTEND_ACCEL
            end
            
        case 2 % EXTEND_ACCEL: Accelerate upward (+x) toward 80% cutoff limit (pct_lim2_low)
            tau_cmd = tau_curr;
            % Trigger braking if position reaches 80% mark OR if required stopping distance reaches 97% cushion
            if (x >= x_lim2_low) || (x + d_stop_worst >= x_lim1_high - x_safety_cushion)
                state_mode = 3; % Transition to EXTEND_DECEL
            end
            
        case 3 % EXTEND_DECEL: Full active regenerative braking until v = 0
            tau_cmd = -I_max * Kt; % Command maximum reverse braking torque
            if v <= 0.0
                v = 0.0;
                state_mode = 4;    % Transition to JOG_TO_LIM1_HIGH
            end
            
        case 4 % JOG_TO_LIM1_HIGH: Crawl gently to 97% upper outer limit with smooth stop ramp
            dist_to_high = x_lim1_high - x;
            if dist_to_high <= 5e-5 && abs(v) <= 2e-3
                x           = x_lim1_high;
                v           = 0.0;
                tau_cmd     = 0.0;
                dwell_timer = 0.0;
                state_mode  = 5;   % Transition to DWELL_AT_LIM1_HIGH
            else
                % Proportional position-approach velocity curve (caps at v_crawl, ramps linearly to 0 at target)
                v_ref   = min(v_crawl, max(0.0, k_pos_crawl * dist_to_high));
                err_v   = v_ref - v;
                F_crawl = k_vel_track * err_v + c_damp * v_ref + F_nut_drag * tanh(v_ref / 0.002);
                tau_cmd = F_crawl / (2 * rad_per_m * eta_screw);
            end
            
        case 5 % DWELL_AT_LIM1_HIGH: Stationary pause at 97% upper outer limit
            tau_cmd     = 0.0;
            v           = 0.0;
            x           = x_lim1_high;
            dwell_timer = dwell_timer + dt;
            if dwell_timer >= t_dwell
                dwell_timer = 0.0;
                state_mode  = 6; % Transition to RETRACT_ACCEL
            end
            
        case 6 % RETRACT_ACCEL: Accelerate downward (-x) toward 20% cutoff limit (pct_lim2_high)
            tau_cmd = -tau_curr;
            % Trigger braking if position reaches 20% mark OR if required stopping distance reaches 3% cushion
            if (x <= x_lim2_high) || (x - d_stop_worst <= x_lim1_low + x_safety_cushion)
                state_mode = 7; % Transition to RETRACT_DECEL
            end
            
        case 7 % RETRACT_DECEL: Full active regenerative braking until v = 0
            tau_cmd = +I_max * Kt; % Command maximum positive braking torque
            if v >= 0.0
                v = 0.0;
                state_mode = 8;    % Transition to JOG_TO_LIM1_LOW
            end
            
        case 8 % JOG_TO_LIM1_LOW: Crawl gently to 3% lower outer limit with smooth stop ramp
            dist_to_low = x - x_lim1_low;
            if dist_to_low <= 5e-5 && abs(v) <= 2e-3
                x           = x_lim1_low;
                v           = 0.0;
                tau_cmd     = 0.0;
                dwell_timer = 0.0;
                state_mode  = 1;   % Transition to DWELL_AT_LIM1_LOW for next cycle
                cycle_count = cycle_count + 1;
                
                % Step up test torque command for next cycle unless hardware limit was already reached
                if ~hit_limit
                    tau_curr = tau_curr + tau_step;
                else
                    % Completed full extension + retraction cycle at the hardware limit; end simulation
                    last_idx = k - 1;
                    break;
                end
            else
                % Proportional position-approach velocity curve (caps at -v_crawl, ramps linearly to 0 at target)
                v_ref   = -min(v_crawl, max(0.0, k_pos_crawl * dist_to_low));
                err_v   = v_ref - v;
                F_crawl = k_vel_track * err_v + c_damp * v_ref + F_nut_drag * tanh(v_ref / 0.002);
                tau_cmd = F_crawl / (2 * rad_per_m * eta_screw);
            end
    end
    
    % --- 4-Quadrant 3-Phase FOC Motor Dynamics & Hardware Saturation ---
    w_motor = v * rad_per_m;             % Motor shaft angular velocity [rad/s] (signed)
    E_back  = Kt * w_motor;              % Motor back-EMF voltage [V] (signed)
    I_cmd   = tau_cmd / Kt;              % Demanded quadrature phase current (I_q) [A]
    
    % SVPWM bus voltage ceiling limits using 3-phase effective resistance (signed) [A]
    I_volt_max = (V_eff - E_back) / R_eff;
    I_volt_min = (-V_eff - E_back) / R_eff;
    
    % Supply power limit ceiling per motor: R_eff * I_q^2 + |E_back| * |I_q| <= P_mot_max (1000 W)
    % Only applies during motoring (when current and back-EMF have the same sign, drawing power from supply)
    is_motoring = (I_cmd * E_back >= 0);
    I_pwr_lim   = (-abs(E_back) + sqrt(E_back^2 + 4 * R_eff * P_mot_max)) / (2 * R_eff);
    
    % Apply active current clamping across all hardware limits (Current, Voltage, Power)
    if I_cmd >= 0
        if is_motoring
            I_upper = min([I_cmd, I_max, I_volt_max, I_pwr_lim]);
        else
            I_upper = min([I_cmd, I_max, I_volt_max]);
        end
        I_actual = max(0, I_upper);
    else
        if is_motoring
            I_lower = max([I_cmd, -I_max, I_volt_min, -I_pwr_lim]);
        else
            I_lower = max([I_cmd, -I_max, I_volt_min]);
        end
        I_actual = min(0, I_lower);
    end
    
    % Detect which hardware limit first bottlenecks active acceleration (State 2 or State 6)
    is_current_clamped = (abs(I_actual) >= I_max - 0.05);
    is_power_clamped   = is_motoring && (abs(I_actual) >= I_pwr_lim - 0.05) && (I_pwr_lim < abs(I_cmd));
    is_volt_clamped    = is_motoring && (abs(I_actual) >= abs(I_volt_max) - 0.05) && (abs(I_volt_max) < abs(I_cmd));
    
    if (is_current_clamped || is_power_clamped || is_volt_clamped) && (state_mode == 2 || state_mode == 6) && ~limit_captured
        hit_limit = true;
        v_at_first_limit = abs(v);
        limit_captured   = true;
        if is_power_clamped
            P_hit = 2 * ((I_actual^2 * R_eff) + abs(E_back * I_actual));
            limit_type_str = sprintf('2 kW System Power Limit (%.1f W reached)', P_hit);
        elseif is_current_clamped
            limit_type_str = sprintf('Controller Current Limit (%.1f A reached)', I_max);
        else
            limit_type_str = sprintf('SVPWM Bus Voltage Limit (%.1f V eff reached)', V_eff);
        end
    end
    
    % Actual electromagnetic torque delivered per motor [N*m]
    tau_delivered = Kt * I_actual;
    
    % Bidirectional electrical power:
    %   Positive (+) = Electrical power drawn from 48V supply during acceleration
    %   Negative (-) = Regenerative electrical power pushed to bus during active braking
    P_motor_elec = (I_actual^2 * R_eff) + (E_back * I_actual);
    P_total_sys  = 2 * P_motor_elec;
    
    % --- Mechanical Force Balance & Equation of Motion ---
    % Ideal linear thrust from both motors before screw losses: F = 2 * tau * (2*pi / p_lead)
    F_motor_ideal = 2 * (tau_delivered * rad_per_m);
    
    % Smoothly transition transmission efficiency between forward driving (* eta_screw)
    % and back-driven braking (/ eta_screw, where screw friction aids deceleration)
    pwr_mech_sign = tanh((v * F_motor_ideal) / 5.0);
    eta_effective = 0.5 * (1 + pwr_mech_sign) * eta_screw + 0.5 * (1 - pwr_mech_sign) * (1 / eta_screw);
    F_drive       = F_motor_ideal * eta_effective;
    
    % Resisting forces: Nut preload/rail seal drag, shock viscous damping, and gravity
    F_friction    = F_nut_drag * tanh(v / 0.002);    % Smooth Coulomb drag [N]
    F_damper      = c_damp * v;                      % Shock viscous damping force [N]
    F_gravity     = m_carriage * g_axial;            % Axial gravity force [N] (0 for horizontal)
    
    % Semi-Implicit (Symplectic) Euler Kinematic Integration
    if state_mode == 1 || state_mode == 5
        a = 0.0;
        v = 0.0;
    else
        a = (F_drive - F_damper - F_friction - F_gravity) / M_eff;
        v = v + a * dt;
        x = x + v * dt;
    end
    
    % Safety verification check for physical overtravel beyond 0% or 100% hard stops
    if x > x_top || x < x_bottom
        warning('Overtravel beyond shock stroke: x = %.2f mm (Limits: 0 to %.2f mm)', ...
            (x - x_bottom) * 1000, shock_stroke_m * 1000);
    end
    
    % Record time-step telemetry
    t_vec(k)     = t;
    x_vec(k)     = x;
    v_vec(k)     = v;
    a_vec(k)     = a;
    I_vec(k)     = I_actual;
    tau_vec(k)   = tau_delivered;
    P_sys_vec(k) = P_total_sys;
    state_vec(k) = state_mode;
end

% Truncate pre-allocated arrays to actual simulation duration
t_vec     = t_vec(1:last_idx);
x_vec     = x_vec(1:last_idx);
v_vec     = v_vec(1:last_idx);
a_vec     = a_vec(1:last_idx);
I_vec     = I_vec(1:last_idx);
tau_vec   = tau_vec(1:last_idx);
P_sys_vec = P_sys_vec(1:last_idx);
state_vec = state_vec(1:last_idx);

%% 5. COMMAND WINDOW SUMMARY OUTPUT
v_max_overall = max(abs(v_vec));
fprintf('\n=======================================================\n');
fprintf('             SHOCK DYNAMOMETER RUN SUMMARY              \n');
fprintf('=======================================================\n');
fprintf('  Damping Constant (c)        : %8.1f N*s/m\n', c_damp);
fprintf('  Shock Total Stroke          : %8.1f mm\n', shock_stroke_m * 1000);
fprintf('  Set 1 Outer Limits (%g%%, %g%%): %8.2f to %.2f mm\n', ...
        pct_lim1_low * 100, pct_lim1_high * 100, ...
        (x_lim1_low - x_bottom) * 1000, (x_lim1_high - x_bottom) * 1000);
fprintf('  Set 2 Inner Limits (%g%%, %g%%): %8.2f to %.2f mm\n', ...
        pct_lim2_low * 100, pct_lim2_high * 100, ...
        (x_lim2_low - x_bottom) * 1000, (x_lim2_high - x_bottom) * 1000);
fprintf('  Effective Moving Mass (M_eff): %7.1f kg (Reflected Inertia Included)\n', M_eff);
fprintf('  Total Full Cycles Completed : %8d\n', cycle_count);
fprintf('  Max Velocity Reached        : %8.3f m/s  (%6.1f mm/s)\n', v_max_overall, v_max_overall * 1000);
fprintf('  Peak Supply Power Drawn     : %8.1f W\n', max(P_sys_vec));
fprintf('  Peak Regen Braking Power    : %8.1f W\n', min(P_sys_vec));
if limit_captured
    fprintf('  Velocity When Limit Hit     : %8.3f m/s  (%6.1f mm/s)\n', v_at_first_limit, v_at_first_limit * 1000);
    fprintf('  Bottleneck Encountered      : %s\n', limit_type_str);
else
    fprintf('  Velocity When Limit Hit     : [None - Limits Not Exceeded within t_max]\n');
end
fprintf('  Travel Range Experienced    : %8.2f to %.2f mm (Allowed: 0 to %.2f mm)\n', ...
        (min(x_vec) - x_bottom) * 1000, (max(x_vec) - x_bottom) * 1000, shock_stroke_m * 1000);
fprintf('=======================================================\n\n');

%% 6. PLOTTING RESULTS (5 Synchronized Telemetry Subplots)
fig = figure('Name', sprintf('Shock Dynamometer (Stroke = %.1f mm, c = %g Ns/m)', shock_stroke_m * 1000, c_damp), ...
       'Color', 'w', 'Position', [80, 30, 1150, 960]);
try theme(fig, 'light'); catch; end

tiled = tiledlayout(5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(5, 1);

% 1. Position relative to shock stroke [mm]
ax(1) = nexttile;
h_pos  = plot(t_vec, (x_vec - x_bottom) * 1000, 'LineWidth', 1.8, 'Color', [0, 0.447, 0.741]); hold on;
h_hard = yline([0, shock_stroke_m * 1000], 'r--', 'LineWidth', 1.1);
h_lim1 = yline([(x_lim1_low - x_bottom) * 1000, (x_lim1_high - x_bottom) * 1000], '--', ...
               'Color', [0.8, 0, 0.8], 'LineWidth', 1.1);
h_lim2 = yline([(x_lim2_low - x_bottom) * 1000, (x_lim2_high - x_bottom) * 1000], ':', ...
               'Color', [0.25, 0.25, 0.25], 'LineWidth', 1.2);

ylabel('Stroke [mm]', 'FontWeight', 'bold');
title(sprintf('Stroke = %.1f mm, c = %g N\\cdots/m', shock_stroke_m * 1000, c_damp), 'FontSize', 11);
ylim([-0.03 * shock_stroke_m * 1000, 1.04 * shock_stroke_m * 1000]);
yticks('auto');
grid on; box on;
set(gca, 'XTickLabel', []);

legend([h_pos, h_hard(1), h_lim1(1), h_lim2(1)], ...
       {'Carriage Position', sprintf('Hard Limits (0 / %.1f mm)', shock_stroke_m * 1000), ...
        sprintf('Set 1 Limits (%g%% / %g%%)', pct_lim1_low * 100, pct_lim1_high * 100), ...
        sprintf('Set 2 Limits (%g%% / %g%%)', pct_lim2_low * 100, pct_lim2_high * 100)}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off', 'FontSize', 8.5);

% 2. Linear Velocity [m/s]
ax(2) = nexttile;
plot(t_vec, v_vec, 'LineWidth', 1.6, 'Color', [0.85, 0.325, 0.098]); hold on;
yline(0, 'k:', 'LineWidth', 1.0, 'Alpha', 0.5);
ylabel('Velocity [m/s]', 'FontWeight', 'bold');
grid on; box on;
set(gca, 'XTickLabel', []);

% 3. Linear Acceleration [m/s^2]
ax(3) = nexttile;
plot(t_vec, a_vec, 'LineWidth', 1.3, 'Color', [0.466, 0.674, 0.188]); hold on;
yline(0, 'k:', 'LineWidth', 1.0, 'Alpha', 0.5);
ylabel('Accel [m/s^2]', 'FontWeight', 'bold');
grid on; box on;
set(gca, 'XTickLabel', []);

% 4. Motor Quadrature Phase Current (I_q) [A]
ax(4) = nexttile;
h_cur  = plot(t_vec, I_vec, 'LineWidth', 1.5, 'Color', [0, 0.447, 0.741]); hold on;
h_clim = yline([-I_max, I_max], 'r--', 'LineWidth', 1.1);
yline(0, 'k:', 'LineWidth', 1.0, 'Alpha', 0.4);
ylabel('Current [A]', 'FontWeight', 'bold');
ylim([-I_max * 1.18, I_max * 1.18]);
grid on; box on;
set(gca, 'XTickLabel', []);
legend([h_cur, h_clim(1)], {'Phase Current (I_q)', sprintf('\\pmI_{max} Limit (\\pm%d A)', round(I_max))}, ...
       'Location', 'northeast', 'Box', 'off', 'FontSize', 8.5);

% 5. Net System Electrical Power (+Motoring Supply Draw / -Regenerative Braking) [W]
ax(5) = nexttile;
h_pwr  = plot(t_vec, P_sys_vec, 'LineWidth', 1.5, 'Color', [0.85, 0.325, 0.098]); hold on;
h_plim = yline(P_sys_max, 'r--', 'LineWidth', 1.1);
yline(0, 'k:', 'LineWidth', 1.0, 'Alpha', 0.5);
ylabel('Power [W]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
ylim([min(-500, min(P_sys_vec) * 1.15), P_sys_max * 1.15]);
grid on; box on;
legend([h_pwr, h_plim(1)], {'Net System Power (+Draw / -Regen)', sprintf('P_{sys,max} Limit (%d W)', round(P_sys_max))}, ...
       'Location', 'northeast', 'Box', 'off', 'FontSize', 8.5);

% Synchronize time axes across all 5 subplots
linkaxes(ax, 'x');
xlim(ax(1), [0, t_vec(end)]);
