package com.airbrakesplugin;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;

/**
 * AirbrakeController
 * Bang-bang controller with dwell (anti-chatter) for binary aerodynamics.
 * - If predicted apogee > target + tol  → latch OPEN and command 1.0
 * - If predicted apogee < target - tol  → latch CLOSED and command 0.0
 * - Enforces minimum dwell in each state so the actuator reaches the endpoint.
 * - Uses a coast-only predictor; subtracts gravity from measured az if available.
 */
public final class AirbrakeController {

    private static final Logger LOG = LoggerFactory.getLogger(AirbrakeController.class);

    // ── config ─────────────────────────────────────────────────────────────
    private final AirbrakeConfig config;

    // ── context hooks (optional) ───────────────────────────────────────────
    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        void switch_altitude_back_to_pressure();
    }
    private ControlContext context;
    public void setContext(ControlContext ctx) { this.context = ctx; }

    // ── controller state ───────────────────────────────────────────────────
    private boolean airbrakesExtended = false; // latched command (true=open)
    private double  lastSwitchTime_s  = 0.0;   // sim time at last state change
    private double  lastTime_s        = 0.0;   // for internal timing if needed

    // Exposed for debug/telemetry
    private double lastPredictedApogee_m = Double.NaN;

    // ── tuning (dwell / thresholds) ────────────────────────────────────────
    // Enough time to actually hit 1.0 and generate binary aero effect
    private static final double MIN_OPEN_DWELL_S   = 0.60;
    private static final double MIN_CLOSED_DWELL_S = 0.40;

    // Thrust gating threshold (N). If thrust data not available, we allow updates.
    private static final double THRUST_MIN_N = 1.0;

    public AirbrakeController(final AirbrakeConfig cfg) {
        this.config = cfg;
    }

    /**
     * Returns the commanded deployment fraction in [0,1].
     *
     * @param alt_m    altitude [m]
     * @param vz_mps   vertical velocity [m/s] (+ upward)
     * @param az_mps2  vertical acceleration [m/s^2] (world Z). If NaN, we fallback.
     * @param mach     current Mach
     * @param deploy   current actuator state [0..1] (for info; latching is time-based)
     * @param dt_s     simulation step [s]
     * @param status   SimulationStatus (for time & optional thrust/mach/apogee reads)
     */
    public double getCommandedDeployment(final double alt_m, final double vz_mps, final double az_mps2, final double mach, final double deploy, final double dt_s, final SimulationStatus status) {
        final double t = (status != null) ? finiteOr(status.getSimulationTime(), lastTime_s + dt_s) : (lastTime_s + dt_s);
        lastTime_s = t;

        // ── Always-open override ───────────────────────────────────────────
        if (isTrue(config.isAlwaysOpenMode())) {
            double pct = clamp01(config.getAlwaysOpenPercentage());
            if (LOG.isTraceEnabled()) LOG.trace("[CTRL] always-open override = {}", pct);
            return pct;
        }

        // ── Safety gates (disable deployment) ──────────────────────────────
        final double machMax = finiteOr(config.getMaxMachForDeployment(), 0.0);
        if (machMax > 0.0 && mach > machMax) {
            // If too fast, latch closed
            latchClosedIfNeeded(t, "Mach gate");
            return 0.0;
        }
        final double deployAltMin = finiteOr(config.getDeployAltitudeThreshold(), 0.0);
        if (alt_m < deployAltMin) {
            latchClosedIfNeeded(t, "Deploy altitude gate");
            return 0.0;
        }

        // ── Thrust gating: don't make *new* decisions during powered phase ─
        final boolean thrusting = isThrusting(status);
        // We still output the *latched* command, but avoid switching while thrusting.
        final double tol_m = config.getApogeeToleranceMeters();
        final double target = finiteOr(config.getTargetApogee(), 0.0);

        // ── Predict apogee (coast-aware, gravity-subtracted) ───────────────
        // Prefer a simple ballistic rise under effective decel (g + a_drag),
        // where a_drag = max(0, -(az + g)). If az is NaN, assume no drag.
        final double g = status.getFlightDataBranch().getLast(FlightDataType.TYPE_GRAVITY);
        double a_drag = 0.0;
        if (Double.isFinite(az_mps2)) {
            a_drag = Math.max(0.0, -(az_mps2 + g));   // remove gravity; count only drag decel
        }
        final double g_eff = Math.max(1e-3, g + a_drag);
        final double vup   = Math.max(0.0, vz_mps);   // only upward velocity contributes to apogee rise
        double ballisticApogee = alt_m + (vup * vup) / (2.0 * g_eff);

        // If OR provides a predictor/estimate, we can fuse it lightly (optional).
        final double orAp = tryORApogee(status);
        final double predictedApogee_m = Double.isFinite(orAp)
                ? 0.5 * ballisticApogee + 0.5 * orAp
                : ballisticApogee;

        lastPredictedApogee_m = predictedApogee_m;

        if (LOG.isDebugEnabled()) {
            LOG.debug("[CTRL] t={} alt={} vz={} az={} mach={} -> a_drag={} g_eff={} predApogee={} target={} tol={} latchedOpen={}",
                    round3(t), round3(alt_m), round3(vz_mps), round3(az_mps2), round3(mach),
                    round3(a_drag), round3(g_eff), round3(predictedApogee_m), round3(target), round3(tol_m),
                    airbrakesExtended);
        }

        // ── Bang-bang with dwell (anti-chatter) ────────────────────────────
        if (!thrusting) {
            final double sinceSwitch = t - lastSwitchTime_s;

            if (!airbrakesExtended) {
                // Consider OPEN if we are above the upper band and have dwelled closed long enough
                if (predictedApogee_m > target + tol_m && sinceSwitch >= MIN_CLOSED_DWELL_S) {
                    airbrakesExtended = true;
                    lastSwitchTime_s = t;
                    if (context != null) context.extend_airbrakes();
                    if (LOG.isInfoEnabled()) LOG.info("[CTRL] OPEN latch @ t={} (pred {} > target+tol {})",
                            round3(t), round3(predictedApogee_m), round3(target + tol_m));
                }
            } else {
                // Consider CLOSE if we are below the lower band and have dwelled open long enough
                if (predictedApogee_m < target - tol_m && sinceSwitch >= MIN_OPEN_DWELL_S) {
                    airbrakesExtended = false;
                    lastSwitchTime_s = t;
                    if (context != null) {
                        context.retract_airbrakes();
                        context.switch_altitude_back_to_pressure();
                    }
                    if (LOG.isInfoEnabled()) LOG.info("[CTRL] CLOSE latch @ t={} (pred {} < target-tol {})",
                            round3(t), round3(predictedApogee_m), round3(target - tol_m));
                }
            }
        } else if (LOG.isTraceEnabled()) {
            LOG.trace("[CTRL] thrusting; holding latched state {}", airbrakesExtended ? "OPEN" : "CLOSED");
        }

        // ── Command: hard 0/1 (listener will rate-limit) ───────────────────
        return airbrakesExtended ? 1.0 : 0.0;
    }

    // ── public helpers for logs/tests ──────────────────────────────────────
    public boolean isLatchedOpen() { return airbrakesExtended; }
    public double  getLastPredictedApogee_m() { return lastPredictedApogee_m; }

    // ── internal utilities ─────────────────────────────────────────────────
    private void latchClosedIfNeeded(final double t, final String reason) {
        if (airbrakesExtended) {
            airbrakesExtended = false;
            lastSwitchTime_s  = t;
            if (context != null) {
                context.retract_airbrakes();
                context.switch_altitude_back_to_pressure();
            }
            if (LOG.isInfoEnabled()) LOG.info("[CTRL] CLOSE latch due to {} @ t={}", reason, round3(t));
        }
    }

    private static boolean isTrue(boolean b) { return b; }

    private static double finiteOr(double v, double dflt) {
        return Double.isFinite(v) ? v : dflt;
    }

    private static double clamp01(double x) {
        return Math.min(1.0, Math.max(0.0, x));
    }

    private static double round3(double x) {
        if (!Double.isFinite(x)) return x;
        return Math.round(x * 1000.0) / 1000.0;
    }

    private boolean isThrusting(final SimulationStatus status) {
        if (status == null || status.getFlightDataBranch() == null) return false;
        try {
            double thrust = status.getFlightDataBranch().getLast(FlightDataType.TYPE_THRUST_FORCE);
            return Double.isFinite(thrust) && thrust > THRUST_MIN_N;
        } catch (Throwable ignore) { /* field may not exist */ }
        return false;
    }

    /**
     * Try to read an apogee estimate from OR (various builds name it differently).
     * Returns NaN if unavailable.
     */
    private double tryORApogee(final SimulationStatus st) {
        if (st == null || st.getFlightDataBranch() == null) return Double.NaN;
        final String[] candidates = new String[] {
                "TYPE_APOGEE_ESTIMATE", "TYPE_APOGEE_PREDICTION", "TYPE_APOGEE"
        };
        for (String name : candidates) {
            Double v = tryFlightDatum(st, name);
            if (v != null) return v;
        }
        return Double.NaN;
    }

    private static Double tryFlightDatum(final SimulationStatus st, final String constName) {
        try {
            java.lang.reflect.Field f = FlightDataType.class.getField(constName);
            Object ft = f.get(null);
            if (ft instanceof FlightDataType) {
                double val = st.getFlightDataBranch().getLast((FlightDataType) ft);
                return Double.isFinite(val) ? val : null;
            }
        } catch (Throwable ignore) {}
        return null;
    }
}
