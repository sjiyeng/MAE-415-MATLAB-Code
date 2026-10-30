# Shock Absorber Dynamometer Physics Simulation (MAE 415)
**North Carolina State University — Pack Motorsports Baja**

This folder contains the MATLAB time-domain physics simulation and automated verification suite for the dual-ballscrew, dual-BLDC linear shock absorber dynamometer.

---

## 1. Repository Files

| File / Folder | Purpose |
| :--- | :--- |
| [`Acceleration_Sim.m`](file:///c:/Users/Srinivas/Documents/MATLAB/MAE%20415/Acceleration_Sim.m) | Main standalone MATLAB simulation script. Simulates a stepped-torque test profile on a single shock configuration and generates a 5-subplot telemetry figure. |
| [`run_and_save_all_cases.m`](file:///c:/Users/Srinivas/Documents/MATLAB/MAE%20415/run_and_save_all_cases.m) | Automated batch test runner. Runs all 4 verification test cases sequentially and exports 300 DPI PNG plots into `physics sim outpout plots/`. |
| [`physics sim outpout plots/`](file:///c:/Users/Srinivas/Documents/MATLAB/MAE%20415/physics%20sim%20outpout%20plots) | Output directory containing exported telemetry plots for each test case. |

---

## 2. Quick Start: Constants to Change for Test Cases

To test a different shock absorber in [`Acceleration_Sim.m`](file:///c:/Users/Srinivas/Documents/MATLAB/MAE%20415/Acceleration_Sim.m), open the script and modify **lines 37–38** under `%% 1. SYSTEM PARAMETERS`:

```matlab
% --- Shock Damper & Test Travel (CHANGE THESE FOR TEST CASES) ---
c_damp          = 100;              % Shock viscous damping coefficient [N*s/m] (e.g., 100 or 4000)
shock_stroke_m  = 8.0 * 0.0254;     % Working stroke of the shock [m] (e.g., 2.0*0.0254 or 8.0*0.0254)
```

### Other Key Tunable Parameters in `Acceleration_Sim.m`

| Variable | Line | Default Value | Description |
| :--- | :---: | :---: | :--- |
| `is_horizontal` | 32 | `true` | Dynamometer orientation (`true` = horizontal with zero axial gravity; `false` = vertical). |
| `pct_lim1_low` / `pct_lim1_high` | 46–47 | `0.03` / `0.97` | **Set 1 (Outer Deadzone Limits):** $3\%$ and $97\%$ turnaround boundaries protecting the $0\%$ and $100\%$ hard stops. |
| `pct_lim2_low` / `pct_lim2_high` | 50–51 | `0.80` / `0.20` | **Set 2 (Inner Cutoff Limits):** $80\%$ extension and $20\%$ retraction cutoffs where acceleration transitions to active braking. |
| `t_dwell` | 59 | `0.20` s | Stationary pause time at the $3\%$ and $97\%$ outer limits before each stroke. |
| `p_lead` | 62 | `0.010` m/rev | Ballscrew lead ($10\text{ mm}$ per revolution). |
| `eta_screw` | 63 | `0.85` | Ballscrew mechanical efficiency ($85\%$). |
| `KV` | 80 | `90` RPM/V | Eaglepower 8308 motor velocity constant ($K_t = \frac{60}{2\pi \cdot KV} \approx 0.1061\text{ N}\cdot\text{m/A}$). |
| `R_phase` | 82 | `0.092` $\Omega$ | Motor datasheet line-to-line winding resistance ($R_{LL}$). |
| `V_bus` | 84 | `48.0` V | DC power supply bus voltage. |
| `I_max` | 89 | `40.0` A | Moteus controller peak phase current limit per motor. |
| `P_sys_max` | 90 | `2000.0` W | Total $48\text{ V}$ DC power supply capacity ($1000\text{ W}$ per motor). |

---

## 3. Physics Model & Governing Equations

### 3.1 Effective Moving Mass & Reflected Rotational Inertia
Because a small linear travel of the carriage ($p_{\text{lead}} = 10\text{ mm}$) requires a full $2\pi\text{ rad}$ rotation of the motors and ballscrews, the rotational inertia of the spinning components dominates the linear dynamics:

$$\text{rad\_per\_m} = \frac{2\pi}{p_{\text{lead}}} \approx 628.32\text{ rad/m}$$

$$M_{\text{eff}} = m_{\text{carriage}} + 2\left(J_{\text{motor}} + J_{\text{screw}}\right)\left(\frac{2\pi}{p_{\text{lead}}}\right)^2$$

With $m_{\text{carriage}} = 1.5\text{ kg}$, $J_{\text{motor}} = 2.5\times 10^{-4}\text{ kg}\cdot\text{m}^2$, and $J_{\text{screw}} = 1.18\times 10^{-5}\text{ kg}\cdot\text{m}^2$, the multiplier $\left(\frac{2\pi}{p_{\text{lead}}}\right)^2 \approx 394,784\text{ rad}^2/\text{m}^2$ yields an effective linear mass of **$M_{\text{eff}} \approx 208.2\text{ kg}$**.

---

### 3.2 3-Phase FOC Motor & Electrical Saturation Model
At every $0.5\text{ ms}$ timestep, the demanded quadrature phase current $I_{\text{cmd}} = \frac{\tau_{\text{cmd}}}{K_t}$ is clamped against three simultaneous hardware envelopes:

1. **Controller Phase Current Limit:**
   $$|I_q| \le I_{\text{max}} = 40.0\text{ A}$$
2. **SVPWM Bus Voltage Ceiling:**
   Using a $95\%$ Space Vector PWM max modulation factor ($V_{\text{eff}} = 0.95 V_{\text{bus}} = 45.6\text{ V}$), back-EMF $E_{\text{back}} = K_t \omega_{\text{motor}}$, and 3-phase effective resistance $R_{\text{eff}} = \frac{3}{4} R_{LL} = 0.069\,\Omega$:
   $$I_{\text{volt,max}} = \frac{V_{\text{eff}} - E_{\text{back}}}{R_{\text{eff}}}, \qquad I_{\text{volt,min}} = \frac{-V_{\text{eff}} - E_{\text{back}}}{R_{\text{eff}}}$$
3. **2 kW Power Supply Limit ($P_{\text{mot,max}} = 1000\text{ W}$ per motor during motoring):**
   Solving $R_{\text{eff}} I_q^2 + |E_{\text{back}}| |I_q| \le P_{\text{mot,max}}$ yields:
   $$I_{\text{pwr,lim}} = \frac{-|E_{\text{back}}| + \sqrt{E_{\text{back}}^2 + 4 R_{\text{eff}} P_{\text{mot,max}}}}{2 R_{\text{eff}}}$$

The net system electrical power is logged bidirectionally as:
$$P_{\text{sys}} = 2 \left(I_{\text{actual}}^2 R_{\text{eff}} + E_{\text{back}} I_{\text{actual}}\right)$$
where **$P_{\text{sys}} > 0$** is power drawn from the $48\text{ V}$ supply during acceleration and **$P_{\text{sys}} < 0$** is regenerative braking power returned to the bus during active deceleration.

---

### 3.3 Mechanical Force Balance & Kinematic Integration
The ideal linear force from both motors is $F_{\text{motor,ideal}} = 2 \tau_{\text{delivered}} \left(\frac{2\pi}{p_{\text{lead}}}\right)$.
* **During Motoring ($v \cdot F_{\text{motor,ideal}} > 0$):** Ballscrew friction reduces forward driving thrust ($F_{\text{drive}} = F_{\text{motor,ideal}} \cdot \eta_{\text{screw}}$).
* **During Active Braking ($v \cdot F_{\text{motor,ideal}} < 0$):** Ballscrew friction opposes motion and aids deceleration ($F_{\text{drive}} = F_{\text{motor,ideal}} / \eta_{\text{screw}}$).

Instantaneous carriage acceleration is integrated at $2\text{ kHz}$ ($\Delta t = 0.5\text{ ms}$) using Semi-Implicit (Symplectic) Euler integration:
$$a(t) = \frac{F_{\text{drive}} - c_{\text{damp}} v - F_{\text{nut,drag}}\tanh(v/0.002) - m_{\text{carriage}} g_{\text{axial}}}{M_{\text{eff}}}$$
$$v(t + \Delta t) = v(t) + a(t)\Delta t, \qquad x(t + \Delta t) = x(t) + v(t + \Delta t)\Delta t$$

---

### 3.4 Model-Free Dynamic Stopping Distance (`d_stop_worst`)
Because the dynamometer controller does not know the shock absorber's damping coefficient $c_{\text{damp}}$ prior to testing, it continuously computes a worst-case ($c_{\text{damp}} = 0$) stopping distance from current velocity $v$:
$$d_{\text{stop,worst}}(v) = \frac{M_{\text{eff}} v^2}{2 \left(F_{\text{max,brake}} + F_{\text{nut,drag}}\right)}$$
Whenever $x + d_{\text{stop,worst}}$ reaches within $1\text{ mm}$ of the $97\%$ upper limit (or $x - d_{\text{stop,worst}}$ reaches within $1\text{ mm}$ of the $3\%$ lower limit), the state machine immediately transitions to full active braking, guaranteeing zero overextension or overretraction across all damping values.

---

## 4. Trajectory State Machine (8 States)

1. **`State 1 (DWELL_AT_LIM1_LOW)`:** Hold stationary at the $3\%$ lower limit for $0.20\text{ s}$.
2. **`State 2 (EXTEND_ACCEL)`:** Accelerate upward ($+x$) with $+\tau_{\text{curr}}$ until reaching the $80\%$ cutoff mark or until $x + d_{\text{stop,worst}}$ reaches the $97\%$ safety cushion.
3. **`State 3 (EXTEND_DECEL)`:** Command full reverse braking torque ($-I_{\text{max}} K_t$) until $v = 0$.
4. **`State 4 (JOG_TO_LIM1_HIGH)`:** Crawl gently at $40\text{ mm/s}$ toward the $97\%$ upper limit and smoothly ramp velocity to $0\text{ m/s}$ upon arrival.
5. **`State 5 (DWELL_AT_LIM1_HIGH)`:** Hold stationary at the $97\%$ upper limit for $0.20\text{ s}$.
6. **`State 6 (RETRACT_ACCEL)`:** Accelerate downward ($-x$) with $-\tau_{\text{curr}}$ until reaching the $20\%$ cutoff mark or until $x - d_{\text{stop,worst}}$ reaches the $3\%$ safety cushion.
7. **`State 7 (RETRACT_DECEL)`:** Command full positive braking torque ($+I_{\text{max}} K_t$) until $v = 0$.
8. **`State 8 (JOG_TO_LIM1_LOW)`:** Crawl gently at $-40\text{ mm/s}$ toward the $3\%$ lower limit, ramp velocity smoothly to $0\text{ m/s}$, increment $\tau_{\text{curr}} \leftarrow \tau_{\text{curr}} + \tau_{\text{step}}$, and repeat until a hardware limit is reached.

---

## 5. Verification Test Matrix Results

Running `run_and_save_all_cases.m` executes all four PDR test cases:

| Test Case | Stroke | Damping ($c$) | Target Velocity | Max Velocity Reached | Travel Range Experienced | Allowed Boundaries | Status |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Case 1** | $2.0''$ ($50.8\text{ mm}$) | $4000\text{ N}\cdot\text{s/m}$ | $\ge 600\text{ mm/s}$ | **$606.0\text{ mm/s}$** | $1.52\text{ to }49.28\text{ mm}$ | $0\text{ to }50.80\text{ mm}$ | **PASS** |
| **Case 2** | $8.0''$ ($203.2\text{ mm}$) | $4000\text{ N}\cdot\text{s/m}$ | $\ge 600\text{ mm/s}$ | **$638.4\text{ mm/s}$** | $6.10\text{ to }197.10\text{ mm}$ | $0\text{ to }203.20\text{ mm}$ | **PASS** |
| **Case 3** | $2.0''$ ($50.8\text{ mm}$) | $100\text{ N}\cdot\text{s/m}$ | $\ge 600\text{ mm/s}$ | **$683.1\text{ mm/s}$** | $1.52\text{ to }49.28\text{ mm}$ | $0\text{ to }50.80\text{ mm}$ | **PASS** |
| **Case 4** | $8.0''$ ($203.2\text{ mm}$) | $100\text{ N}\cdot\text{s/m}$ | $\ge 600\text{ mm/s}$ | **$683.2\text{ mm/s}$** | $6.10\text{ to }197.10\text{ mm}$ | $0\text{ to }203.20\text{ mm}$ | **PASS** |
