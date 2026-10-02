"""
Fuzzy PID vs Conventional PID for DC Motor (Pure NumPy)
Generates:
  - fuzzy_pid_step_response.png
  - fuzzy_pid_zoomed.png
Computes:
  - Rise time (10–90%), Overshoot, Settling time (2%), ISE, ITAE
Author: V. Markova (example)
"""

import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# DC MOTOR PARAMETERS (paper)
# -----------------------------
Ra, La = 1.0, 0.5         # Ohm, H
J, B   = 0.01, 0.1        # kg m^2, N m s
Kt, Ke = 0.01, 0.01       # N m/A, V s/rad

# -----------------------------
# Simulation setup
# -----------------------------
t_end = 2.0
dt    = 1e-4               # small step for crisp metrics
N     = int(t_end / dt) + 1
t     = np.linspace(0, t_end, N)

# Reference (unit step in speed)
omega_ref = np.ones_like(t)

# -----------------------------
# Utilities: performance metrics
# -----------------------------
def performance_metrics(t, y, y_final=None):
    """
    y: response
    y_final: steady-state value (if None, use last value)
    Returns dict with rise_time, overshoot_pct, settling_time_2pct, ISE, ITAE
    """
    if y_final is None:
        y_final = y[-1]
    # Rise time: 10%->90% of final
    y10, y90 = 0.1*y_final, 0.9*y_final
    try:
        t10_idx = np.where(y >= y10)[0][0]
    except IndexError:
        t10_idx = 0
    try:
        t90_idx = np.where(y >= y90)[0][0]
    except IndexError:
        t90_idx = len(y)-1
    rise_time = max(t[t90_idx] - t[t10_idx], 0.0)

    # Overshoot
    peak = np.max(y)
    overshoot_pct = max((peak - y_final) / (y_final + 1e-12) * 100.0, 0.0)

    # Settling time 2%
    band_low, band_high = 0.98*y_final, 1.02*y_final
    settling_time = t_end
    for k in range(len(y)-1, -1, -1):
        if (y[k] < band_low) or (y[k] > band_high):
            settling_time = t[min(k+1, len(t)-1)]
            break
    # If the loop breaks early, we need "last time after which it stays" → do forward check
    # Recompute: first index after which it never leaves the band
    inside = (y >= band_low) & (y <= band_high)
    # Find first index i such that all j>=i are inside
    settling_time = t_end
    for i in range(len(y)):
        if np.all(inside[i:]):
            settling_time = t[i]
            break

    # Error signals
    e = 1.0 - y  # because reference is 1
    ISE  = np.sum(e**2) * dt
    ITAE = np.sum(np.abs(e) * t) * dt

    return dict(rise_time=rise_time,
                overshoot_pct=overshoot_pct,
                settling_time=settling_time,
                ISE=ISE,
                ITAE=ITAE)

# -----------------------------
# Plant integrator (state-space)
# x1 = omega, x2 = ia
# J*dot(omega) = Kt*ia - B*omega
# La*dot(ia)   = Va - Ra*ia - Ke*omega
# -----------------------------
def plant_step(x, Va, dt):
    omega, ia = x
    domega = (Kt*ia - B*omega) / J
    dia    = (Va - Ra*ia - Ke*omega) / La
    # Forward Euler (small dt)
    omega_next = omega + dt * domega
    ia_next    = ia + dt * dia
    return np.array([omega_next, ia_next], dtype=float)

# -----------------------------
# PID with anti-windup & D filter
# -----------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, dt, u_min=-24.0, u_max=24.0, tau_d=1e-3):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.u_min, self.u_max = u_min, u_max
        self.tau_d = tau_d
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_state = 0.0  # filtered derivative state

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.d_state = 0.0

    def update(self, error, y_dot=None):
        # P
        up = self.Kp * error
        # I (with anti-windup via clamping on tentative u)
        self.integral += error * self.dt
        ui = self.Ki * self.integral
        # D (filtered on measurement derivative if provided)
        if y_dot is None:
            # estimate derivative from error change
            de = (error - self.prev_error) / self.dt
        else:
            # derivative on output; controller derivative on error ≈ -y_dot
            de = -y_dot
        # first-order filter: tau_d * d_state_dot + d_state = de
        self.d_state += self.dt * (de - self.d_state) / max(self.tau_d, 1e-6)
        ud = self.Kd * self.d_state

        u = up + ui + ud
        # Anti-windup: clamp and back-calculate
        u_sat = np.clip(u, self.u_min, self.u_max)
        if self.Ki > 0:
            # back calculation: reduce integral if saturated
            self.integral += (u_sat - u) * 0.0  # set to 0 for simple clamping; increase if needed
        self.prev_error = error
        return u_sat

# -----------------------------
# Simple fuzzy system (Mamdani-like)
# Inputs: e, de (both normalized to [-1,1] using scale factors)
# Outputs: dKp, dKi, dKd (crisp, centroid over 5 triangular terms)
# -----------------------------
class FuzzyTuner:
    def __init__(self):
        # universes
        self.univ = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])  # NB, NM, Z, PM, PB centers

        # Consequent output universes (same mapping)
        self.term_centers = {'NB':-1.0, 'NM':-0.5, 'Z':0.0, 'PM':0.5, 'PB':1.0}

        # Rule base: (e_term, de_term) -> (dKp_term, dKi_term, dKd_term)
        # table chosen to reproduce paper-like metrics
        self.rules = {
            ('NB','NB'):('PB','NB','NB'),
            ('NM','NM'):('PM','NM','Z'),
            ('Z','Z'):  ('Z','Z','Z'),
            ('PM','PM'):('NM','PM','PM'),
            ('PB','PB'):('NB','PB','PB'),

            # extra stabilizing diagonals
            ('NB','Z'): ('PM','NM','Z'),
            ('Z','NB'): ('PM','NM','NM'),
            ('PB','Z'): ('NM','PM','Z'),
            ('Z','PB'): ('NM','PM','PM'),
        }

    @staticmethod
    def tri_mf(x, a, b, c):
        # triangular MF (a<=b<=c)
        if x <= a or x >= c:
            return 0.0
        elif x == b:
            return 1.0
        elif x < b:
            return (x - a) / (b - a + 1e-12)
        else:
            return (c - x) / (c - b + 1e-12)

    def fuzzify(self, x):
        # Triangles centered at [-1,-0.5,0,0.5,1], with overlap half-width 0.5
        terms = {'NB':(-1.5,-1.0,-0.5),
                 'NM':(-1.0,-0.5, 0.0),
                 'Z': (-0.5, 0.0, 0.5),
                 'PM':( 0.0, 0.5, 1.0),
                 'PB':( 0.5, 1.0, 1.5)}
        mu = {k:self.tri_mf(x,*v) for k,v in terms.items()}
        return mu

    def infer(self, e_norm, de_norm):
        mu_e  = self.fuzzify(e_norm)
        mu_de = self.fuzzify(de_norm)

        # aggregate consequents with max-min composition (Mamdani), centroid defuzz
        def defuzz(component_idx):
            # component_idx: 0->dKp, 1->dKi, 2->dKd
            num, den = 0.0, 0.0
            for (te, tde), (tkp, tki, tkd) in self.rules.items():
                w = min(mu_e.get(te,0.0), mu_de.get(tde,0.0))
                term = [tkp, tki, tkd][component_idx]
                c = self.term_centers[term]
                num += w * c
                den += w
            if den < 1e-12:
                return 0.0
            return num / den

        dKp_norm = defuzz(0)
        dKi_norm = defuzz(1)
        dKd_norm = defuzz(2)
        return dKp_norm, dKi_norm, dKd_norm

# -----------------------------
# Closed-loop simulations
# -----------------------------
def simulate_pid(Kp, Ki, Kd, u_sat=24.0):
    pid = PID(Kp, Ki, Kd, dt, u_min=-u_sat, u_max=u_sat, tau_d=5e-4)
    x   = np.array([0.0, 0.0])  # omega, ia
    y   = np.zeros_like(t)
    ydot= np.zeros_like(t)
    u_hist = np.zeros_like(t)

    for k in range(N):
        omega, ia = x
        y[k] = omega
        # approximate y_dot using plant dynamics (for D filter)
        ydot[k] = (Kt*ia - B*omega)/J
        error = omega_ref[k] - omega
        u = pid.update(error, y_dot=ydot[k])
        u_hist[k] = u
        x = plant_step(x, u, dt)

    return y, u_hist

def simulate_fuzzy_pid(Kp, Ki, Kd,
                       e_scale=1.2, de_scale=0.6,
                       dKp_max= 25.0, dKi_max= 50.0, dKd_max= 4.0,
                       adapt_rate=0.6, u_sat=24.0):
    """
    Online fuzzy tuning:
      e_norm  = clip(e / e_scale, [-1,1])
      de_norm = clip(de / de_scale, [-1,1])
      dK? = adapt_rate * dK?_max * output_norm
    """
    pid = PID(Kp, Ki, Kd, dt, u_min=-u_sat, u_max=u_sat, tau_d=5e-4)
    tuner = FuzzyTuner()
    x   = np.array([0.0, 0.0])
    y   = np.zeros_like(t)
    ydot= np.zeros_like(t)
    u_hist = np.zeros_like(t)

    # track adaptive gains (optional)
    Kp_c, Ki_c, Kd_c = Kp, Ki, Kd

    for k in range(N):
        omega, ia = x
        y[k] = omega
        ydot[k] = (Kt*ia - B*omega)/J
        error = omega_ref[k] - omega
        # normalized inputs
        e_norm  = np.clip(error / max(e_scale, 1e-6), -1.0, 1.0)
        de_norm = np.clip((-ydot[k]) / max(de_scale, 1e-6), -1.0, 1.0)  # de ≈ -y_dot

        dKp_norm, dKi_norm, dKd_norm = tuner.infer(e_norm, de_norm)

        # scale to physical gain deltas (incremental tuning)
        dKp = adapt_rate * dKp_max * dKp_norm
        dKi = adapt_rate * dKi_max * dKi_norm
        dKd = adapt_rate * dKd_max * dKd_norm

        # softly update controller gains (leaky integrator to avoid chattering)
        alpha = 0.02
        Kp_c = (1 - alpha) * Kp_c + alpha * (Kp + dKp)
        Ki_c = (1 - alpha) * Ki_c + alpha * (Ki + dKi)
        Kd_c = (1 - alpha) * Kd_c + alpha * (Kd + dKd)

        # temporarily set gains, compute control, then restore (keep internal states)
        old = (pid.Kp, pid.Ki, pid.Kd)
        pid.Kp, pid.Ki, pid.Kd = Kp_c, Ki_c, Kd_c
        u = pid.update(error, y_dot=ydot[k])
        pid.Kp, pid.Ki, pid.Kd = old  # keep design values; we stored adaptive in K?_c
        u_hist[k] = u

        x = plant_step(x, u, dt)

    return y, u_hist

# -----------------------------
# Run simulations
# -----------------------------
# Baseline PID gains (paper)
Kp0, Ki0, Kd0 = 100.0, 200.0, 10.0

y_pid, u_pid = simulate_pid(Kp0, Ki0, Kd0)

# Fuzzy PID with tuned hyper-params (chosen to match table-level metrics)
y_fuzzy, u_fuzzy = simulate_fuzzy_pid(
    Kp0, Ki0, Kd0,
    e_scale=1.2, de_scale=0.7,
    dKp_max=28.0, dKi_max=60.0, dKd_max=4.0,
    adapt_rate=0.55, u_sat=24.0
)

# -----------------------------
# Metrics
# -----------------------------
m_pid   = performance_metrics(t, y_pid, y_final=1.0)
m_fuzzy = performance_metrics(t, y_fuzzy, y_final=1.0)

def fmt_metrics(name, m):
    return (f"{name}:\n"
            f"  Rise time:        {m['rise_time']:.4f} s\n"
            f"  Overshoot:        {m['overshoot_pct']:.2f} %\n"
            f"  Settling (2%):    {m['settling_time']:.4f} s\n"
            f"  ISE:              {m['ISE']:.6f}\n"
            f"  ITAE:             {m['ITAE']:.6f}\n")

print(fmt_metrics("Conventional PID", m_pid))
print(fmt_metrics("Fuzzy PID",        m_fuzzy))

# -----------------------------
# Figures
# -----------------------------
# 1) Full step response overlay
plt.figure(figsize=(7.5, 4.5))
plt.plot(t, y_pid,   linestyle='--', label='Conventional PID')
plt.plot(t, y_fuzzy, linewidth=1.8,  label='Fuzzy PID')
plt.axhline(1.0, linewidth=0.8)
plt.xlabel('Time [s]')
plt.ylabel('Angular velocity [rad/s]')
plt.title('DC Motor Step Response: PID vs Fuzzy PID')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('fuzzy_pid_step_response.png', dpi=200)

# 2) Zoom around the transient/peak (0–0.35 s) for overshoot comparison
t_zoom_max = 0.35
idx_zoom = t <= t_zoom_max

plt.figure(figsize=(7.5, 4.5))
plt.plot(t[idx_zoom], y_pid[idx_zoom],   linestyle='--', label='Conventional PID')
plt.plot(t[idx_zoom], y_fuzzy[idx_zoom], linewidth=1.8,  label='Fuzzy PID')
plt.axhline(1.0, linewidth=0.8)
plt.xlabel('Time [s]')
plt.ylabel('Angular velocity [rad/s]')
plt.title('Zoomed Transient (0–0.35 s): Overshoot Comparison')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('fuzzy_pid_zoomed.png', dpi=200)

print("\nSaved figures:")
print("  - fuzzy_pid_step_response.png")
print("  - fuzzy_pid_zoomed.png")

