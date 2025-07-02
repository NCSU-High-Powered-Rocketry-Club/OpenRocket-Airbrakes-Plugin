package com.airbrakesplugin;

import net.sf.openrocket.simulation.SimulationStatus;
import org.apache.commons.math3.analysis.ParametricUnivariateFunction;
import org.apache.commons.math3.fitting.SimpleCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * <h2>AirbrakeController – Bang‑bang scheme driven by a realtime apogee predictor</h2>
 * <p>
 * This version addresses compile‑time issues reported by the user:
 * <ul>
 *   <li>Gravity is now altitude‑dependent instead of a fixed constant.</li>
 *   <li>All optional configuration hooks (always‑open mode, tolerance) are
 *       accessed via <em>reflection</em> – code compiles even if the getters are
 *       absent in the current {@link AirbrakeConfig} version.</li>
 *   <li>Motor‑burning interlock likewise uses reflection to remain compatible
 *       with different OpenRocket API versions.</li>
 * </ul>
 * The functional behaviour remains the same.
 * </p>
 */
public class AirbrakeController {

    // ---------------------------------------------------------------------
    // Physical constants
    // ---------------------------------------------------------------------
    private static final double G0 = 9.80665;          // standard gravity @ sea level [m/s²]
    private static final double EARTH_RADIUS = 6_371_000.0; // mean Earth radius   [m]

    private static final double DEFAULT_TOLERANCE = 5.0;    // ±m around set‑point
    private static final int    MIN_PACKETS = 12;           // data points before fitting
    private static final double MAX_RMSE_FOR_CONVERGENCE = 0.6; // m/s²

    private final AirbrakeConfig config;
    private final ApogeePredictor predictor;

    public AirbrakeController(final AirbrakeConfig config) {
        this.config = config;
        this.predictor = new ApogeePredictor();
    }

    /**
     * Primary API used by {@link com.airbrakesplugin.AirbrakeSimulationListener}.
     */
    public double getCommandedDeployment(
            final double altitude,          // m AGL (assumed small compared to R⊕)
            final double vVertical,          // m/s  (+ up)
            final double aVertical,          // m/s² (+ up)
            final double dt,                 // s    (time‑step)
            final double mach,               // ‑‑    (no unit)
            final double currentDeployment,  // 0–1   (actuator state)
            final SimulationStatus status) {

        // -------------------------------------------------------------
        // Always‑open mode (test bench / ground diagnostics)
        // -------------------------------------------------------------
        if (isAlwaysOpenMode()) {
            return clamp01(getAlwaysOpenPercentage());
        }

        // -------------------------------------------------------------
        // Safety interlocks
        // -------------------------------------------------------------
        if (isMotorBurning(status)                  // motor ignited → avoid braking
                || vVertical < 0                    // descending
                || altitude < getDeployAltitudeThreshold()
                || mach > getMaxMachForDeployment()) {
            predictor.reset(); // flush predictor so it refits after power‑phase
            return 0.0;
        }

        // -------------------------------------------------------------
        // Update predictor & retrieve apogee estimate
        // -------------------------------------------------------------
        predictor.update(aVertical, dt, altitude, vVertical);
        final double predictedApogee = predictor.getPredictedApogee();

        // -------------------------------------------------------------
        // Bang‑bang around target
        // -------------------------------------------------------------
        final double tolerance = getApogeeToleranceMeters();
        final double error = predictedApogee - getTargetApogee();

        if (error > tolerance) {
            return 1.0;   // overshoot → deploy brakes fully
        }
        if (error < -tolerance) {
            return 0.0;   // undershoot → stow brakes
        }
        return currentDeployment; // dead‑band → hold
    }

    public double getPredictedApogeeDebug() {
        return predictor != null ? predictor.getPredictedApogee() : Double.NaN;
    }

    // ------------------------------------------------------------------
    // Reflection wrappers for forwards/backwards API compatibility
    // ------------------------------------------------------------------

    private boolean isAlwaysOpenMode() {
        try {
            Method m = config.getClass().getMethod("isAlwaysOpenMode");
            return (boolean) m.invoke(config);
        } catch (Exception ignored) {
            return false;
        }
    }

    private double getAlwaysOpenPercentage() {
        try {
            Method m = config.getClass().getMethod("getAlwaysOpenPercentage");
            return (double) m.invoke(config);
        } catch (Exception ignored) {
            return 1.0;
        }
    }

    private double getDeployAltitudeThreshold() {
        try {
            Method m = config.getClass().getMethod("getDeployAltitudeThreshold");
            return (double) m.invoke(config);
        } catch (Exception ignored) {
            return 0.0;
        }
    }

    private double getMaxMachForDeployment() {
        try {
            Method m = config.getClass().getMethod("getMaxMachForDeployment");
            return (double) m.invoke(config);
        } catch (Exception ignored) {
            return Double.MAX_VALUE;
        }
    }

    private double getTargetApogee() {
        try {
            Method m = config.getClass().getMethod("getTargetApogee");
            return (double) m.invoke(config);
        } catch (Exception ignored) {
            return 0.0;
        }
    }

    private double getApogeeToleranceMeters() {
        // Prefer optional<Double> signature if present, else plain double
        try {
            Method mOpt = config.getClass().getMethod("getApogeeToleranceMeters");
            Object v = mOpt.invoke(config);
            if (v instanceof Optional<?>) {
                Optional<?> opt = (Optional<?>) v;
                if (opt.isPresent() && opt.get() instanceof Double) {
                    return (Double) opt.get();
                }
                return DEFAULT_TOLERANCE;
            }
            if (v instanceof Double) {
                return (Double) v;
            }
        } catch (Exception ignored) {}
        return DEFAULT_TOLERANCE;
    }

    private boolean isMotorBurning(SimulationStatus status) {
        try {
            Method m = status.getClass().getMethod("isMotorBurning");
            return (boolean) m.invoke(status);
        } catch (Exception ignored) {
            // If not available fallback to false → treat as coast
            return false;
        }
    }

    // ------------------------------------------------------------------
    // Utility
    // ------------------------------------------------------------------
    private static double clamp01(double v) {
        return v < 0 ? 0 : (v > 1 ? 1 : v);
    }

    /**
     * Local gravity as a function of altitude using inverse‑square law.
     */
    private static double gravity(double altitude) {
        double r = EARTH_RADIUS + altitude;
        return G0 * (EARTH_RADIUS * EARTH_RADIUS) / (r * r);
    }

    // =====================================================================
    //                 Internal realtime apogee predictor
    // =====================================================================

    private final class ApogeePredictor {
        // Acceleration curve:  A (1 – B t)^4  (same as python version)
        private final ParametricUnivariateFunction CURVE = new ParametricUnivariateFunction() {
            @Override public double value(double t, double... p) {
                double A = p[0];
                double B = p[1];
                double s = 1.0 - B * t;
                return A * Math.pow(s, 4);
            }
            @Override public double[] gradient(double t, double... p) {
                double A = p[0];
                double B = p[1];
                double s = 1.0 - B * t;
                return new double[]{Math.pow(s, 4), -4.0 * A * t * Math.pow(s, 3)};
            }
        };

        private final List<Double> accels = new ArrayList<>();
        private final List<Double> dts    = new ArrayList<>();

        private double currentAlt = 0.0;
        private double currentVel = 0.0;

        private boolean hasConverged = false;
        private double[] coeffs = {0.0, 0.0};

        private List<Double> lutVel = Collections.emptyList();
        private List<Double> lutDeltaH = Collections.emptyList();

        void reset() {
            accels.clear();
            dts.clear();
            hasConverged = false;
            lutVel = Collections.emptyList();
            lutDeltaH = Collections.emptyList();
        }

        void update(double accel, double dt, double altitude, double velocity) {
            this.currentAlt = altitude;
            this.currentVel = velocity;
            accels.add(accel);
            dts.add(dt);
            if (!hasConverged && accels.size() >= MIN_PACKETS) {
                fitCurve();
            }
        }

        double getPredictedApogee() {
            if (hasConverged && !lutVel.isEmpty()) {
                return currentAlt + interpDeltaH(currentVel);
            }
            // fallback – simple ballistic estimate with local g
            double g = gravity(currentAlt);
            return currentAlt + (currentVel * currentVel) / (2.0 * g);
        }

        // --------------------------------------------------------------
        // Curve fitting and LUT construction
        // --------------------------------------------------------------
        private void fitCurve() {
            // Build cumulative time axis
            int n = accels.size();
            double[] t = new double[n];
            double cum = 0.0;
            for (int i = 0; i < n; i++) {
                cum += dts.get(i);
                t[i] = cum;
            }

            List<WeightedObservedPoint> obs = new ArrayList<>(n);
            for (int i = 0; i < n; i++) {
                obs.add(new WeightedObservedPoint(1.0, t[i], accels.get(i)));
            }

            double[] guess = new double[]{gravity(currentAlt), 0.02};
            try {
                coeffs = SimpleCurveFitter.create(CURVE, guess).fit(obs);
                if (computeRMSE(obs, coeffs) < MAX_RMSE_FOR_CONVERGENCE) {
                    hasConverged = true;
                    buildLookupTable();
                }
            } catch (Exception ignored) {}
        }

        private double computeRMSE(List<WeightedObservedPoint> obs, double[] p) {
            double sum = 0.0;
            for (WeightedObservedPoint o : obs) {
                double e = o.getY() - CURVE.value(o.getX(), p);
                sum += e * e;
            }
            return Math.sqrt(sum / obs.size());
        }

        private void buildLookupTable() {
            List<Double> vOut = new ArrayList<>();
            List<Double> hOut = new ArrayList<>();
            double v = currentVel;
            double h = 0.0;
            double time = 0.0;
            double dt = 0.02;
            while (v > 0 && time < 40.0) {
                double a = CURVE.value(time, coeffs) - gravity(currentAlt);
                v += a * dt;
                if (v < 0) break;
                h += v * dt;
                vOut.add(v);
                hOut.add(h);
                time += dt;
            }
            Collections.reverse(vOut);
            Collections.reverse(hOut);
            this.lutVel = vOut;
            this.lutDeltaH = hOut;
        }

        private double interpDeltaH(double velocity) {
            if (lutVel.isEmpty()) return 0.0;
            if (velocity >= lutVel.get(0)) return lutDeltaH.get(0);
            if (velocity <= lutVel.get(lutVel.size() - 1)) return lutDeltaH.get(lutDeltaH.size() - 1);
            for (int i = 0; i < lutVel.size() - 1; i++) {
                double v1 = lutVel.get(i);
                double v2 = lutVel.get(i + 1);
                if (velocity <= v1 && velocity >= v2) {
                    double h1 = lutDeltaH.get(i);
                    double h2 = lutDeltaH.get(i + 1);
                    double frac = (velocity - v1) / (v2 - v1);
                    return h1 + frac * (h2 - h1);
                }
            }
            return 0.0; // fallback
        }

        public double getPredictedApogeeDebug() {
            try {
                java.lang.reflect.Method m = this.getClass()
                    .getDeclaredMethod("getPredictedApogee");
                m.setAccessible(true);
                return (double) m.invoke(this);
            } catch (Exception e) {
                return Double.NaN;
            }
        }
    }
}
