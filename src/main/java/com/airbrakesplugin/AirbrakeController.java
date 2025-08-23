package com.airbrakesplugin;

import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.util.Coordinate;

import org.apache.commons.math3.analysis.ParametricUnivariateFunction;
import org.apache.commons.math3.fitting.SimpleCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * AirbrakeController (OpenRocket 24.12 compatible)
 * - Reads state via SimulationStatus (core API)
 * - Detects thrust using FlightData.getLast(FlightDataType.TYPE_THRUST_FORCE)
 * - Predicts apogee with curve-fit + ballistic fallback
 * - Bang-bang with deadband around a target apogee
 */
public final class AirbrakeController {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    public interface ControlContext {
        default void extend_airbrakes() {}
        default void retract_airbrakes() {}
        /** Optional hook: restore altitude source to pressure after retract */
        default void switch_altitude_back_to_pressure() {}
    }

    private final AirbrakeConfig config;
    private ControlContext context;

    // State
    private Double lastVz = null;     // previous-step vertical speed (m/s)
    private Double lastTime = null;   // previous-step sim time (s)
    private boolean airbrakesExtended = false;
    private Coordinate lastVel = null; // Add this field to store previous velocity vector

    // Predictor
    private final ApogeePredictor predictor = new ApogeePredictor();

    // Tuning
    private static final int    MIN_PACKETS_FOR_FIT = 15;
    private static final double MAX_RMSE_FOR_CONV   = 1.2;   // m/s^2
    private static final double SMOOTH_ALPHA        = 0.5;   // EMA smoothing

    // Telemetry
    private double lastPredictedApogee = Double.NaN;

    public AirbrakeController(final AirbrakeConfig config) {
        this.config = config;
    }

    public void setContext(ControlContext ctx) {
        this.context = ctx;
    }

    /**
     * Returns the commanded deployment fraction in [0,1].
     * Bang-bang with deadband: extend if predicted apogee > target + tol; retract if < target - tol.
     * Inside [target - tol, target + tol], hold current state.
     *
     */
    public double getCommandedDeployment(final double alt_m,
                                         final double vz_input,
                                         final double azFromOR_mps2,
                                         final double mach,
                                         final double currentDeployment,
                                         final double dt,
                                         final SimulationStatus status) {

        final boolean debugEnabled = (log != null && log.isDebugEnabled());
        final double t = finiteOr(status != null ? status.getSimulationTime() : Double.NaN, 0.0);
        
        // --- Read kinematics (24.12 SimulationStatus getters) ---
        final Coordinate vVec = (status != null) ? status.getRocketVelocity() : null;
        Coordinate vNow = vVec;  // Create an alias for clarity
        
        // Calculate acceleration if not provided by OR
        Coordinate aVec = null;
        if (vNow != null &&  Double.isFinite(vNow.x) && Double.isFinite(vNow.y) && Double.isFinite(vNow.z) && lastVel != null && dt > 1e-6) {
            double ax = (vNow.x - lastVel.x) / dt;
            double ay = (vNow.y - lastVel.y) / dt;
            double az = status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_Z);
            aVec = new Coordinate(ax, ay, az);
            if (debugEnabled) {
                log.debug("Calculated acceleration: ({}, {}, {})", ax, ay, az);
            }
        }
        // Save current velocity for next step
        lastVel = (vNow != null) ? vNow : lastVel;
        

        // World-Z vertical speed (prefer SimulationStatus over the input hint); pitch only for logs
        double vz   = vz_input;
        double vmag = 0.0;
        double pitchDeg = 0.0;
        if (vVec != null && Double.isFinite(vVec.x) && Double.isFinite(vVec.y) && Double.isFinite(vVec.z)) {
            vmag = vVec.length();
            vz   = vVec.z;
            if (vmag > 1e-6) {
                final double cosToZ = clamp(vz / vmag, -1.0, 1.0);
                pitchDeg = Math.toDegrees(Math.acos(cosToZ));
            }
        }

        // ---------- SAFETY GATES ----------
        if (vz <= 0.0) { // descending
            airbrakesExtended = false;
            updateLastState(vz, t);
            return 0.0;
        }
        if (config.getDeployAltitudeThreshold() > 0.0 && alt_m < config.getDeployAltitudeThreshold()) {
            airbrakesExtended = false;
            updateLastState(vz, t);
            return 0.0;
        }
        try {
            final double machCap = config.getMaxMachForDeployment();
            if (machCap > 0.0 && Double.isFinite(mach) && mach > machCap) {
                airbrakesExtended = false;
                updateLastState(vz, t);
                return 0.0;
            }
        } catch (Throwable ignored) { /* older configs might not define this */ }

        // ---------- BUILD DRAG-ONLY VERTICAL DECEL ----------
        // Prefer finite-difference net vertical accel (includes gravity), else fall back to OR accel hints
        double az_net = Double.NaN;
        if (lastVz != null && dt > 1e-6 && Double.isFinite(vz)) {
            az_net = (vz - lastVz) / dt; // includes gravity
        } else if (aVec != null && Double.isFinite(aVec.z)) {
            az_net = aVec.z;             // use OR-provided accel
        } else if (Double.isFinite(azFromOR_mps2)) {
            az_net = azFromOR_mps2;      // last resort
        }

        final double g = gravityAt(alt_m);

        // ---------- COAST-AWARE GATE ----------
        boolean thrusting = false;
        try {
            if (status != null) {
                // OpenRocket 24.12 API uses direct accessor methods instead of getFlightData/getLast pattern
                double thrustN = 0.0;
                
                // Method 1: Try using direct thrust accessor if available
                try {
                    thrustN = status.getFlightDataBranch().getLast(FlightDataType.TYPE_THRUST_FORCE);
                } catch (Throwable e) {
                    // Method 2: Try accessing flight data through new API
                    try {
                        thrustN = status.getFlightDataBranch().getLast(FlightDataType.TYPE_THRUST_FORCE);
                    } catch (Throwable e2) {
                        // Method 3: Try getting last value if available through FlightDataBranch
                        if (status.getFlightDataBranch() != null) {
                            thrustN = status.getFlightDataBranch().getLast(FlightDataType.TYPE_THRUST_FORCE);
                        }
                    }
                }
                
                thrusting = Double.isFinite(thrustN) && thrustN > 1.0;
                
                if (log != null && log.isDebugEnabled()) {
                    log.debug("Thrust detection: thrustN={}, thrusting={}", thrustN, thrusting);
                }
            }
        } catch (Throwable ignored) {
            // Fall back to acceleration-based detection (existing code)
        }

        // Keep the acceleration-based detection as a fallback
        if (!thrusting) {
            // Heuristic fallback: during powered ascent, net vertical accel >> −g
            try {
                thrusting = Double.isFinite(az_net) && (az_net > -0.5 * g);
            } catch (Throwable ignored) {}
        }

        if (thrusting) {
            // Avoid polluting the fit during thrust; hold current state
            updateLastState(vz, t);
            return airbrakesExtended ? openFraction() : 0.0;
        }

        // Coast regime: az_net ≈ −g − drag_z  ⇒ drag magnitude = −(az_net + g)
        double a_drag_vert = 0.0;
        if (vz > 0.0 && Double.isFinite(az_net)) {
            a_drag_vert = Math.max(0.0, -(az_net + g));
        }
        if (a_drag_vert < 0.01) a_drag_vert = 0.0;

        // Smooth the decel before feeding the predictor (EMA)
        final double a_smooth = predictor.hasSamples()
                ? (1.0 - SMOOTH_ALPHA) * predictor.peekLastAccel() + SMOOTH_ALPHA * a_drag_vert
                : a_drag_vert;

        predictor.update(alt_m, vz, a_smooth, dt);

        // ---------- PREDICT APEX ----------
        double predicted = predictor.predictApogee(alt_m, vz, MIN_PACKETS_FOR_FIT, MAX_RMSE_FOR_CONV);
        lastPredictedApogee = predicted;

        // Fallback if the fit is degenerate (NaN, ≈alt, or RMSE≈0 with zero accel)
        final double rmse = predictor.getLastRmse();
        final boolean degenerate = (!Double.isFinite(predicted))
                || (predicted <= alt_m + 0.5)
                || (rmse < 1e-6 && a_smooth == 0.0);

        if (degenerate) {
            final double a_eff = Math.max(0.25, a_smooth); // keep denominator sane
            predicted = alt_m + (vz * vz) / (2.0 * (g + a_eff));
            lastPredictedApogee = predicted;
        }

        final double target = finiteOr(config.getTargetApogee(), 1500.0);
        double tol = 5.0;
        try {
            final Optional<Double> opt = config.getApogeeToleranceMeters();
            if (opt != null && opt.isPresent()) tol = opt.get();
        } catch (Throwable ignored) {}

        // ---------- BANG-BANG WITH DEADBAND ----------
        if (Double.isFinite(predicted) && Double.isFinite(target)) {
            // Extending increases drag ⇒ reduces apogee
            if (!airbrakesExtended && predicted > target + tol) {
                if (context != null) context.extend_airbrakes();
                airbrakesExtended = true;
                log.info("EXTENDING AIRBRAKES: apogee={}, target={}, tol={}", predicted, target, tol);
            } else if (airbrakesExtended && predicted < target - tol) {
                if (context != null) {
                    context.retract_airbrakes();
                    context.switch_altitude_back_to_pressure();
                }
                airbrakesExtended = false;
                log.info("RETRACTING AIRBRAKES: apogee={}, target={}, tol={}", predicted, target, tol);
            }
            // else: within deadband → hold current state
        } else {
            // No reliable prediction yet → keep stowed while climbing
            airbrakesExtended = false;
        }

        if (debugEnabled) {
            log.debug(String.format(
                    "[CTRL] t=%.2f alt=%.1f vz=%.2f pitch=%.1f° aFit=%.3f rmse=%.6f apogee=%.1f target=%.1f tol=%.1f extended=%s",
                    t, alt_m, vz, pitchDeg, a_smooth, rmse, predicted, target, tol, airbrakesExtended));
        }

        updateLastState(vz, t);

        // Commanded fraction (always-open overrides; else full-open when extended)
        return airbrakesExtended ? openFraction() : 0.0;
    }

    /** Latest apogee prediction (m). */
    public double getLastPredictedApogee() {
        return lastPredictedApogee;
    }

    // ───────────────────────────── helpers ────────────────────────────────

    private void updateLastState(double vz, double t) {
        this.lastVz = vz;
        this.lastTime = t;
    }

    private double openFraction() {
        try {
            if (config.isAlwaysOpenMode()) {
                return clamp01(finiteOr(config.getAlwaysOpenPercentage(), 1.0));
            }
        } catch (Throwable ignored) {}
        return 1.0;
    }

    private static double finiteOr(double val, double fallback) {
        return Double.isFinite(val) ? val : fallback;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static double clamp01(double x) {
        return clamp(x, 0.0, 1.0);
    }

    /** Gravity variation with altitude: g0*(Re/(Re+h))^2. */
    private static double gravityAt(double alt_m) {
        final double g0 = 9.80665, Re = 6_371_000.0;
        final double f = Re / (Re + Math.max(0.0, alt_m));
        return g0 * f * f;
    }

    // ─────────────────────── Apogee Predictor ───────────────────────

    private static final class ApogeePredictor {

        // Model a(t) = A * (1 - B t)^4  (positive deceleration magnitude)
        private static final ParametricUnivariateFunction MODEL = new ParametricUnivariateFunction() {
            @Override
            public double value(double t, double[] p) {
                final double A = p[0], B = p[1];
                final double s = Math.max(0.0, 1.0 - B * t);
                return A * Math.pow(s, 4);
            }
            @Override
            public double[] gradient(double t, double[] p) {
                final double A = p[0], B = p[1];
                final double s = Math.max(0.0, 1.0 - B * t);
                final double s3 = Math.pow(s, 3);
                return new double[] { Math.pow(s, 4), -4.0 * A * t * s3 };
            }
        };

        private final List<Double> accels = new ArrayList<>(); // positive decel [m/s^2]
        private final List<Double> dts    = new ArrayList<>(); // step sizes [s]
        private double currentAlt = 0.0;
        private double currentVel = 0.0;

        private boolean hasConverged = false;
        private double[] coeffs = { 0.0, 0.0 }; // A, B
        private double lastRmse = Double.NaN;

        private List<Double> lutVel    = Collections.emptyList(); // ascending v
        private List<Double> lutDeltaH = Collections.emptyList(); // Δh(v) to apex
        private String lastMode = "BALLISTIC";
        private double lastPredictedApogee = Double.NaN;

        void reset() {
            accels.clear();
            dts.clear();
            lutVel = Collections.emptyList();
            lutDeltaH = Collections.emptyList();
            hasConverged = false;
            coeffs[0] = coeffs[1] = 0.0;
            lastRmse = Double.NaN;
            lastMode = "BALLISTIC";
            lastPredictedApogee = Double.NaN;
        }

        boolean hasSamples() { return !accels.isEmpty(); }
        double  peekLastAccel() { return accels.isEmpty() ? 0.0 : accels.get(accels.size() - 1); }
        String  getLastMode() { return lastMode; }
        double  getLastRmse() { return lastRmse; }
        double  getLastPredictedApogee() { return lastPredictedApogee; }

        void update(double alt, double vz, double a_dec_pos, double dt) {
            if (!Double.isFinite(alt) || !Double.isFinite(vz) || !Double.isFinite(a_dec_pos) || dt <= 0.0) return;
            if (vz <= 0.0) return; // only ascent samples
            currentAlt = alt;
            currentVel = vz;
            accels.add(Math.max(0.0, a_dec_pos));
            dts.add(dt);
        }

        double predictApogee(double alt, double vz, int minPackets, double maxRmse) {
            if (!Double.isFinite(alt) || !Double.isFinite(vz)) {
                lastPredictedApogee = Double.NaN;
                lastMode = "BALLISTIC";
                return lastPredictedApogee;
            }

            if (accels.size() >= minPackets) {
                try {
                    // Build time grid and fit A, B
                    final List<WeightedObservedPoint> pts = new ArrayList<>(accels.size());
                    double t = 0.0;
                    for (int i = 0; i < accels.size(); i++) {
                        pts.add(new WeightedObservedPoint(1.0, t, accels.get(i)));
                        t += dts.get(i);
                    }
                    final double A0 = Math.max(0.1, accels.get(0));
                    final double B0 = 0.02;
                    final double[] p = SimpleCurveFitter.create(MODEL, new double[] { A0, B0 }).fit(pts);

                    // RMSE
                    double sumsq = 0.0;
                    t = 0.0;
                    for (int i = 0; i < accels.size(); i++) {
                        final double e = accels.get(i) - MODEL.value(t, p);
                        sumsq += e * e;
                        t += dts.get(i);
                    }
                    final double rmse = Math.sqrt(sumsq / Math.max(1, accels.size()));

                    if (Double.isFinite(rmse) && rmse <= maxRmse && Double.isFinite(p[0]) && Double.isFinite(p[1]) && p[1] >= 0.0) {
                        coeffs = p.clone();
                        hasConverged = true;
                        lastRmse = rmse;
                        buildLut(coeffs);
                    } else {
                        hasConverged = false;
                        lastRmse = rmse;
                    }
                } catch (Throwable fitErr) {
                    hasConverged = false;
                    lastRmse = Double.NaN;
                }
            }

            final double hApex;
            if (hasConverged && !lutVel.isEmpty()) {
                final double dh = interpDeltaH(vz);
                hApex = alt + Math.max(0.0, dh);
                lastMode = "FIT";
            } else {
                final double g = gravityAt(alt);
                hApex = alt + Math.max(0.0, (vz * vz) / (2.0 * g));
                lastMode = "BALLISTIC";
            }

            lastPredictedApogee = hApex;
            return hApex;
        }

        private void buildLut(double[] p) {
            final double A = p[0], B = p[1];
            final double dt = 0.02;
            double v = Math.max(0.0, currentVel);
            double h = 0.0;

            final List<Double> V = new ArrayList<>();
            final List<Double> H = new ArrayList<>();
            V.add(v); H.add(h);

            double t = 0.0;
            for (int i = 0; i < 20000 && v > 1e-4; i++) {
                final double a = Math.max(0.0, MODEL.value(t, new double[] { A, B })); // decel magnitude
                v = Math.max(0.0, v - a * dt); // dv/dt = -a during ascent
                h += v * dt;
                t += dt;

                V.add(v);
                H.add(h);
            }

            lutVel = V;
            lutDeltaH = H;
        }

        private double interpDeltaH(double velocity) {
            if (lutVel.isEmpty()) return 0.0;
            if (velocity <= lutVel.get(0)) return lutDeltaH.get(0);
            if (velocity >= lutVel.get(lutVel.size() - 1)) return lutDeltaH.get(lutDeltaH.size() - 1);

            for (int i = 0; i < lutVel.size() - 1; i++) {
                final double v1 = lutVel.get(i), v2 = lutVel.get(i + 1);
                if (velocity >= v1 && velocity <= v2) {
                    final double h1 = lutDeltaH.get(i), h2 = lutDeltaH.get(i + 1);
                    final double f  = (velocity - v1) / (v2 - v1);
                    return h1 + f * (h2 - h1);
                }
            }
            return lutDeltaH.get(lutDeltaH.size() - 1);
        }
    }
}
