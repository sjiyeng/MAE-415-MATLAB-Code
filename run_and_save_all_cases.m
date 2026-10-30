%% AUTOMATED TEST RUNNER & PLOT EXPORTER FOR SHOCK DYNAMOMETER
% Runs all 4 test cases with 3-Phase FOC + SVPWM & Smooth Crawl Physics
% and exports high-resolution PNG plots to 'physics sim outpout plots' folder.

clear; clc; close all;

out_dir = fullfile(pwd, 'physics sim outpout plots');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

test_cases = [
    struct('name', 'Case1_ShortStroke_HighDamping_2in_c4000', ...
           'desc', 'Case 1: Short Stroke (2.0 in), High Damping (c = 4000 N*s/m)', ...
           'stroke_in', 2.0, 'c_damp', 4000), ...
    struct('name', 'Case2_LongStroke_HighDamping_8in_c4000', ...
           'desc', 'Case 2: Long Stroke (8.0 in), High Damping (c = 4000 N*s/m)', ...
           'stroke_in', 8.0, 'c_damp', 4000), ...
    struct('name', 'Case3_ShortStroke_LowDamping_2in_c100', ...
           'desc', 'Case 3: Short Stroke (2.0 in), Low Damping (c = 100 N*s/m)', ...
           'stroke_in', 2.0, 'c_damp', 100), ...
    struct('name', 'Case4_LongStroke_LowDamping_8in_c100', ...
           'desc', 'Case 4: Long Stroke (8.0 in), Low Damping (c = 100 N*s/m)', ...
           'stroke_in', 8.0, 'c_damp', 100)
];

for i = 1:length(test_cases)
    tc = test_cases(i);
    fprintf('\n=======================================================\n');
    fprintf('RUNNING %s\n', tc.desc);
    fprintf('=======================================================\n');
    
    fig = run_single_simulation(tc.stroke_in * 0.0254, tc.c_damp);
    
    png_path = fullfile(out_dir, [tc.name, '.png']);
    exportgraphics(fig, png_path, 'Resolution', 300, 'BackgroundColor', 'white');
    fprintf('Saved plot to: %s\n', png_path);
    close(fig);
end

fprintf('\nAll 4 test cases successfully executed and plots saved to:\n%s\n\n', out_dir);


%% --- LOCAL SIMULATION FUNCTION ---
function fig = run_single_simulation(shock_stroke_m, c_damp)
    % 1. SYSTEM PARAMETERS
    is_horizontal   = true;
    g_accel         = 9.81;
    g_axial         = is_horizontal * 0 + (~is_horizontal) * g_accel;

    x_start_m       = 0.050;
    x_bottom        = x_start_m;
    x_top           = x_start_m + shock_stroke_m;

    % Two Sets of Deadzones
    pct_lim1_low    = 0.03;
    pct_lim1_high   = 1.00 - pct_lim1_low;
    pct_lim2_low    = 0.80;
    pct_lim2_high   = 0.20;

    x_lim1_low      = x_bottom + pct_lim1_low  * shock_stroke_m;
    x_lim2_low      = x_bottom + pct_lim2_low  * shock_stroke_m;
    x_lim2_high     = x_bottom + pct_lim2_high * shock_stroke_m;
    x_lim1_high     = x_bottom + pct_lim1_high * shock_stroke_m;

    t_dwell         = 0.20;

    % Mechanical Transmission
    p_lead          = 0.010;
    eta_screw       = 0.85;
    m_carriage      = 1.5;
    J_motor         = 2.5e-4;

    d_screw         = 0.014;
    L_screw         = 0.40;
    rho_steel       = 7850;
    m_screw         = rho_steel * (pi * (d_screw/2)^2 * L_screw);
    J_screw         = 0.5 * m_screw * (d_screw/2)^2;

    rad_per_m       = (2 * pi) / p_lead;
    M_eff           = m_carriage + 2 * (J_motor + J_screw) * (rad_per_m^2);

    % Motor & Electrical (3-Phase FOC + SVPWM Modulation)
    KV              = 90;
    Kt              = 60 / (2 * pi * KV);
    R_phase         = 0.092;
    R_eff           = 0.75 * R_phase;
    V_bus           = 48.0;
    eta_svpwm       = 0.95;
    V_eff           = eta_svpwm * V_bus;

    I_max           = 40.0;
    P_sys_max       = 2000.0;
    P_mot_max       = P_sys_max / 2.0;

    F_max_brake     = 2 * (Kt * I_max * rad_per_m / eta_screw);
    F_nut_drag      = 15.0;

    % Profile Configuration
    tau_start       = 0.30;
    tau_step        = 0.35;
    dt              = 0.0005;
    t_max           = 60.0;

    N = round(t_max / dt);
    t_vec     = zeros(N, 1);
    x_vec     = zeros(N, 1);
    v_vec     = zeros(N, 1);
    a_vec     = zeros(N, 1);
    I_vec     = zeros(N, 1);
    tau_vec   = zeros(N, 1);
    P_sys_vec = zeros(N, 1);
    state_vec = zeros(N, 1);

    state_mode      = 1;
    dwell_timer     = 0.0;
    x               = x_lim1_low;
    v               = 0.0;
    tau_curr        = tau_start;
    cycle_count     = 0;
    hit_limit       = false;
    last_idx        = N;

    v_crawl         = 0.040;
    k_pos_crawl     = 15.0;
    k_vel_track     = 8000;

    v_at_first_limit     = NaN;
    limit_type_str       = 'None';
    limit_captured       = false;

    % Integration Loop
    for k = 1:N
        t = (k - 1) * dt;
        d_stop_worst = (M_eff * v^2) / (2 * (F_max_brake + F_nut_drag));
        x_safety_cushion = 0.0010;

        switch state_mode
            case 1 % DWELL_AT_LIM1_LOW
                tau_cmd     = 0.0;
                v           = 0.0;
                x           = x_lim1_low;
                dwell_timer = dwell_timer + dt;
                if dwell_timer >= t_dwell
                    dwell_timer = 0.0;
                    state_mode  = 2;
                end

            case 2 % EXTEND_ACCEL
                tau_cmd = tau_curr;
                if (x >= x_lim2_low) || (x + d_stop_worst >= x_lim1_high - x_safety_cushion)
                    state_mode = 3;
                end

            case 3 % EXTEND_DECEL
                tau_cmd = -I_max * Kt;
                if v <= 0.0
                    v = 0.0;
                    state_mode = 4;
                end

            case 4 % JOG_TO_LIM1_HIGH
                dist_to_high = x_lim1_high - x;
                if dist_to_high <= 5e-5 && abs(v) <= 2e-3
                    x           = x_lim1_high;
                    v           = 0.0;
                    tau_cmd     = 0.0;
                    dwell_timer = 0.0;
                    state_mode  = 5;
                else
                    v_ref   = min(v_crawl, max(0.0, k_pos_crawl * dist_to_high));
                    err_v   = v_ref - v;
                    F_crawl = k_vel_track * err_v + c_damp * v_ref + F_nut_drag * tanh(v_ref / 0.002);
                    tau_cmd = F_crawl / (2 * rad_per_m * eta_screw);
                end

            case 5 % DWELL_AT_LIM1_HIGH
                tau_cmd     = 0.0;
                v           = 0.0;
                x           = x_lim1_high;
                dwell_timer = dwell_timer + dt;
                if dwell_timer >= t_dwell
                    dwell_timer = 0.0;
                    state_mode  = 6;
                end

            case 6 % RETRACT_ACCEL
                tau_cmd = -tau_curr;
                if (x <= x_lim2_high) || (x - d_stop_worst <= x_lim1_low + x_safety_cushion)
                    state_mode = 7;
                end

            case 7 % RETRACT_DECEL
                tau_cmd = +I_max * Kt;
                if v >= 0.0
                    v = 0.0;
                    state_mode = 8;
                end

            case 8 % JOG_TO_LIM1_LOW
                dist_to_low = x - x_lim1_low;
                if dist_to_low <= 5e-5 && abs(v) <= 2e-3
                    x           = x_lim1_low;
                    v           = 0.0;
                    tau_cmd     = 0.0;
                    dwell_timer = 0.0;
                    state_mode  = 1;
                    cycle_count = cycle_count + 1;

                    if ~hit_limit
                        tau_curr = tau_curr + tau_step;
                    else
                        last_idx = k - 1;
                        break;
                    end
                else
                    v_ref   = -min(v_crawl, max(0.0, k_pos_crawl * dist_to_low));
                    err_v   = v_ref - v;
                    F_crawl = k_vel_track * err_v + c_damp * v_ref + F_nut_drag * tanh(v_ref / 0.002);
                    tau_cmd = F_crawl / (2 * rad_per_m * eta_screw);
                end
        end

        % 3-Phase FOC Motor Saturation
        w_motor = v * rad_per_m;
        E_back  = Kt * w_motor;
        I_cmd   = tau_cmd / Kt;

        I_volt_max = (V_eff - E_back) / R_eff;
        I_volt_min = (-V_eff - E_back) / R_eff;

        is_motoring = (I_cmd * E_back >= 0);
        I_pwr_lim   = (-abs(E_back) + sqrt(E_back^2 + 4 * R_eff * P_mot_max)) / (2 * R_eff);

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

        tau_delivered = Kt * I_actual;
        P_motor_elec  = (I_actual^2 * R_eff) + (E_back * I_actual);
        P_total_sys   = 2 * P_motor_elec;

        F_motor_ideal = 2 * (tau_delivered * rad_per_m);
        pwr_mech_sign = tanh((v * F_motor_ideal) / 5.0);
        eta_effective = 0.5 * (1 + pwr_mech_sign) * eta_screw + 0.5 * (1 - pwr_mech_sign) * (1 / eta_screw);
        F_drive       = F_motor_ideal * eta_effective;
        F_friction    = F_nut_drag * tanh(v / 0.002);
        F_damper      = c_damp * v;
        F_gravity     = m_carriage * g_axial;

        if state_mode == 1 || state_mode == 5
            a = 0.0;
            v = 0.0;
        else
            a = (F_drive - F_damper - F_friction - F_gravity) / M_eff;
            v = v + a * dt;
            x = x + v * dt;
        end

        t_vec(k)     = t;
        x_vec(k)     = x;
        v_vec(k)     = v;
        a_vec(k)     = a;
        I_vec(k)     = I_actual;
        tau_vec(k)   = tau_delivered;
        P_sys_vec(k) = P_total_sys;
        state_vec(k) = state_mode;
    end

    t_vec     = t_vec(1:last_idx);
    x_vec     = x_vec(1:last_idx);
    v_vec     = v_vec(1:last_idx);
    a_vec     = a_vec(1:last_idx);
    I_vec     = I_vec(1:last_idx);
    tau_vec   = tau_vec(1:last_idx);
    P_sys_vec = P_sys_vec(1:last_idx);
    state_vec = state_vec(1:last_idx);

    v_max_overall = max(abs(v_vec));
    fprintf('  Damping Constant (c)        : %8.1f N*s/m\n', c_damp);
    fprintf('  Shock Total Stroke          : %8.1f mm\n', shock_stroke_m * 1000);
    fprintf('  Max Velocity Reached        : %8.3f m/s  (%6.1f mm/s)\n', v_max_overall, v_max_overall * 1000);
    fprintf('  Peak Supply Power Drawn     : %8.1f W\n', max(P_sys_vec));
    fprintf('  Peak Regen Braking Power    : %8.1f W\n', min(P_sys_vec));
    fprintf('  Bottleneck Encountered      : %s\n', limit_type_str);
    fprintf('  Travel Range Experienced    : %8.2f to %.2f mm (Allowed: 0 to %.2f mm)\n', ...
            (min(x_vec) - x_bottom) * 1000, (max(x_vec) - x_bottom) * 1000, shock_stroke_m * 1000);

    % Plotting
    fig = figure('Name', sprintf('Shock Dyno (Stroke = %.1f mm, c = %g Ns/m)', shock_stroke_m * 1000, c_damp), ...
           'Color', 'w', 'Position', [80, 30, 1150, 960], 'Visible', 'off');
    try theme(fig, 'light'); catch; end

    tiledlayout(5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(5, 1);

    % 1. Position
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

    % 2. Velocity
    ax(2) = nexttile;
    plot(t_vec, v_vec, 'LineWidth', 1.6, 'Color', [0.85, 0.325, 0.098]); hold on;
    yline(0, 'k:', 'LineWidth', 1.0, 'Alpha', 0.5);
    yline([0.6, -0.6], 'g--', 'LineWidth', 1.0);
    ylabel('Velocity [m/s]', 'FontWeight', 'bold');
    grid on; box on;
    set(gca, 'XTickLabel', []);

    % 3. Acceleration
    ax(3) = nexttile;
    plot(t_vec, a_vec, 'LineWidth', 1.3, 'Color', [0.466, 0.674, 0.188]); hold on;
    yline(0, 'k:', 'LineWidth', 1.0, 'Alpha', 0.5);
    ylabel('Accel [m/s^2]', 'FontWeight', 'bold');
    grid on; box on;
    set(gca, 'XTickLabel', []);

    % 4. Motor Current
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

    % 5. Total Electrical Power (+ Motoring / - Regenerative Braking)
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

    linkaxes(ax, 'x');
    xlim(ax(1), [0, t_vec(end)]);
end
