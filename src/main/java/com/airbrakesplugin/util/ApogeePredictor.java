package com.airbrakesplugin.util;

import com.airbrakesplugin.util.AirDensity;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * RK4-based apogee predictor.
 *
 * Integrates a vertical point-mass with quadratic drag using 4th-order Runge–Kutta:
 *
 *   dz/dt = v
 *   dv/dt = -g - k(z) * v * |v|
 *
 * k(z) is density-scaled: k(z) = k_now * (rho(z)/rho_now), where k_now is calibrated
 * from the current measured vertical acceleration (including g):
 *
 *   a_meas ≈ -g - k_now * v*|v|  →  k_now = max(0, -(a_meas + g)/(v*|v|))
 *
 * Units: meters, seconds. Returns apogee altitude above MSL to match OpenRocket.
 */
public final class ApogeePredictor {

    private static final Logger LOG = LoggerFactory.getLogger(ApogeePredictor.class);

    // Constants
    private static final double G = 9.80665;     // m/s^2
    private static final double MIN_V = 0.01;    // m/s → consider "at apogee"
    private static final int    DEFAULT_STEPS = 30;
    private static final double MIN_DT = 1e-3;
    private static final double MAX_DT = 0.1;    // sec per RK4 substep cap

    // Config
    private int rk4Steps = DEFAULT_STEPS;

    // Last prediction
    private Double lastApogeeMSL = null;

    public ApogeePredictor() { }

    /** Tune number of RK4 substeps per update (default ~30). */
    public void setRk4Steps(int steps) {
        if (steps >= 5 && steps <= 200) this.rk4Steps = steps;
    }

    /** Reset (clears last prediction). */
    public void reset() { lastApogeeMSL = null; }

    /**
     * Update and produce a new apogee estimate.
     *
     * @param a_worldZ_includingG   measured vertical accel (includes g), m/s^2
     * @param dtSim                 outer simulation dt, s (used to size look-ahead step)
     * @param altitudeAGL           current altitude AGL (OpenRocket z), m
     * @param altitudeMSL           current altitude MSL (z + site altitude), m
     * @param vZ                    vertical velocity (world-Z), m/s
     */
    public void update(double a_worldZ_includingG,
                       double dtSim,
                       double altitudeAGL,
                       double altitudeMSL,
                       double vZ) {

        // If descending / nearly stopped: apogee is now.
        if (!Double.isFinite(vZ) || vZ <= 0.0 || Math.abs(vZ) < MIN_V) {
            lastApogeeMSL = altitudeMSL;
            return;
        }
        if (!Double.isFinite(a_worldZ_includingG)) {
            // Ballistic fallback (no drag) if accel is unavailable.
            lastApogeeMSL = altitudeMSL + (vZ * vZ) / (2.0 * G);
            return;
        }

        // Calibrate drag coefficient at "now": a = -g - k_now*v|v|
        final double denom = vZ * Math.abs(vZ);
        double k_now = denom != 0.0 ? -(a_worldZ_includingG + G) / denom : 0.0;
        if (!Double.isFinite(k_now) || k_now < 0.0) k_now = 0.0;

        final double rho_now = AirDensity.rhoISA(altitudeMSL);

        // Use ~30 fixed substeps unless constrained by safety caps.
        final double t_no_drag = vZ / G; // time to apogee ignoring drag
        double dt = Math.max(MIN_DT, Math.min(MAX_DT, Math.abs(t_no_drag) / Math.max(1, rk4Steps)));
        int steps = rk4Steps;

        double z = altitudeMSL;
        double v = vZ;
        double zPrev = z, vPrev = v;
        boolean crossed = false;

        for (int i = 0; i < steps; i++) {
            zPrev = z; vPrev = v;

            final double k1_v = accel(z, v, k_now, rho_now) * dt;
            final double k1_z = v * dt;

            final double v2 = v + 0.5 * k1_v;
            final double z2 = z + 0.5 * k1_z;
            final double k2_v = accel(z2, v2, k_now, rho_now) * dt;
            final double k2_z = v2 * dt;

            final double v3 = v + 0.5 * k2_v;
            final double z3 = z + 0.5 * k2_z;
            final double k3_v = accel(z3, v3, k_now, rho_now) * dt;
            final double k3_z = v3 * dt;

            final double v4 = v + k3_v;
            final double z4 = z + k3_z;
            final double k4_v = accel(z4, v4, k_now, rho_now) * dt;
            final double k4_z = v4 * dt;

            v += (k1_v + 2.0*k2_v + 2.0*k3_v + k4_v) / 6.0;
            z += (k1_z + 2.0*k2_z + 2.0*k3_z + k4_z) / 6.0;

            if (v <= 0.0) { crossed = true; break; }
        }

        if (crossed) {
            // Linear interpolation to v=0 crossing.
            final double frac = vPrev / (vPrev - Math.max(v, -1e-9));
            final double zAp = zPrev + Math.max(0.0, Math.min(1.0, frac)) * (z - zPrev);
            lastApogeeMSL = Double.isFinite(zAp) ? zAp : zPrev;
        } else {
            // If not crossed, conservative ballistic top-off.
            lastApogeeMSL = z + Math.max(0.0, v*v) / (2.0 * G);
        }
    }

    /** dv/dt at (z, v) with density-scaled quadratic drag. */
    private static double accel(double zMSL, double v, double k_now, double rho_now) {
        if (!Double.isFinite(v)) return -G;
        final double rho = AirDensity.rhoISA(zMSL);
        final double k = (rho_now > 0.0) ? k_now * (rho / rho_now) : k_now;
        return -G - k * v * Math.abs(v);
    }

    /** Primary getter used by controller/listener. */
    public Double getPredictionIfReady() { return lastApogeeMSL; }

    /** Backward-compatible alias. */
    public Double getApogeeBestEffort() { return lastApogeeMSL; }

    /** Compatibility shim for the old 4-arg update signature. */
    @Deprecated
    public void update(double a_includingG, double dt, double altitudeAGL, double vZ) {
        // No site altitude in old API → assume MSL ≈ AGL.
        update(a_includingG, dt, altitudeAGL, altitudeAGL, vZ);
    }

    /** No-op for old tracing hook (left to preserve UI toggle). */
    public void setTraceSink(TraceSink sink) { /* no-op */ }

    // TraceSink kept for binary compatibility with prior code
    public interface TraceSink { void trace(String line); }
}
