package com.airbrakesplugin;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.FlightDataType;
import org.apache.commons.math3.analysis.ParametricUnivariateFunction;
import org.apache.commons.math3.fitting.SimpleCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * AirbrakeController – Bang-bang controlled by a realtime apogee predictor.
 *
 * Uses OpenRocket dt and vertical acceleration (if available) or dv/dt fallback.
 * Gates on motor-burning, ascent-only, deploy-altitude threshold, and Mach cap.
 * Fits to NET vertical acceleration and integrates directly (no extra g subtraction).
 * Emits DEBUG logs for predicted apogee, mode (FIT vs BALLISTIC), convergence and gating.
 */
public class AirbrakeController {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    // Physical constants (for ballistic fallback)
    private static final double G0 = 9.80665;
    private static final double EARTH_RADIUS = 6_371_000.0;

    private static final double DEFAULT_TOLERANCE = 10.0;     // ±m deadband
    private static final int    MIN_PACKETS       = 12;       // samples before fit
    private static final double MAX_RMSE_FOR_CONV = 1.2;      // m/s²

    // We fit & integrate NET vertical acceleration in coast (dvz/dt).
    private static final boolean FIT_IS_NET_ACCEL = true;

    private final AirbrakeConfig  config;
    private final ApogeePredictor predictor;

    private Double lastVz = null;      // used only if azFromOR is invalid

    // Toggle logging at runtime if needed
    private boolean debugEnabled = true;

    public AirbrakeController(final AirbrakeConfig config) {
        this.config = config;
        this.predictor = new ApogeePredictor();
    }

    public void setDebugEnabled(boolean enabled) {
        this.debugEnabled = enabled;
    }

    /**
     * Controller entry point.
     */
    public double getCommandedDeployment(
            final double alt,
            final double vz,
            final double azFromOR,
            final double mach,
            final double currentDeployment,
            final double dt,
            final SimulationStatus status) {

        final double h = (Double.isFinite(dt) && dt > 1e-6) ? dt : 1e-2;

        // Always-open diagnostic mode (via reflection, if present)
        if (isAlwaysOpenMode()) {
            if (debugEnabled && log.isDebugEnabled()) {
                log.debug("[AIRBRAKE] Always-open mode active → forcing deployment={}",
                        clamp01(getAlwaysOpenPercentage()));
            }
            return clamp01(getAlwaysOpenPercentage());
        }

        // Safety / interlocks (deploy only in safe, ascending coast)
        final boolean burning = isMotorBurning(status);
        final boolean descending = (vz <= 0.0);
        final double altThresh = getDeployAltitudeThreshold();
        final boolean belowThresh = (alt < altThresh);
        final double machMax = getMaxMachForDeployment();
        final boolean overMach = (mach > machMax);

        if (burning || descending || belowThresh || overMach) {
            predictor.reset(); // keep fit clean
            lastVz = vz;

            if (debugEnabled && log.isDebugEnabled()) {
                log.debug("[AIRBRAKE] Gate/stow: burning={} descending={} belowThresh({}<{})={} overMach({}>{})={} → cmd=0",
                        burning, descending, alt, altThresh, belowThresh, mach, machMax, overMach);
            }
            return 0.0;
        }

        // Compute net vertical acceleration (prefer OR's; else dv/dt)
        final double aVertical = Double.isFinite(azFromOR)
                ? azFromOR
                : (lastVz == null ? 0.0 : (vz - lastVz) / h);
        lastVz = vz;

        // Update predictor and compute apogee
        predictor.update(aVertical, h, alt, vz);

        final double predictedApogee = predictor.getPredictedApogee();
        final String  mode           = predictor.getLastMode();         // "FIT" or "BALLISTIC"
        final boolean converged      = predictor.hasConverged();
        final double  rmse           = predictor.getLastRmse();         // NaN when not available
        final double[] coeffs        = predictor.getLastCoeffs();

        final double tolerance       = getApogeeToleranceMeters();
        final double targetApogee    = getTargetApogee();
        final double error           = predictedApogee - targetApogee;

        // For context, also log OR's own apogee (best-effort)
        final double orApogee = getOpenRocketApogee(status);

        if (debugEnabled && log.isDebugEnabled()) {
            final Object rmseLog = Double.isFinite(rmse) ? rmse : "n/a";
            final Object aLog = (coeffs != null && coeffs.length > 0 && Double.isFinite(coeffs[0])) ? coeffs[0] : "n/a";
            final Object bLog = (coeffs != null && coeffs.length > 1 && Double.isFinite(coeffs[1])) ? coeffs[1] : "n/a";

            log.debug("[APOGEE] mode={} pred={}m OR={}m target={}m tol=±{}m err={}m conv={} rmse={} A={} B={} | alt={} vz={} az={} dt={} M={}",
                    mode,
                    safeD(predictedApogee), safeD(orApogee), safeD(targetApogee), safeD(tolerance), safeD(error),
                    converged, rmseLog, aLog, bLog,
                    safeD(alt), safeD(vz), safeD(aVertical), safeD(h), safeD(mach));
        }

        // Bang-bang with deadband
        if (error >  tolerance) return 1.0; // overshoot → brake
        if (error < -tolerance) return 0.0; // undershoot → stow
        return currentDeployment;           // inside deadband → hold
    }

    public double getPredictedApogeeDebug() {
        return predictor != null ? predictor.getPredictedApogee() : Double.NaN;
    }

    // ───────────────────────────────────────────────────────────────────
    // Reflection wrappers for config compatibility
    // ───────────────────────────────────────────────────────────────────
    private boolean isAlwaysOpenMode() {
        try {
            Method m = config.getClass().getMethod("isAlwaysOpenMode");
            return (boolean) m.invoke(config);
        } catch (Exception ignored) { return false; }
    }

    private double getAlwaysOpenPercentage() {
        try {
            Method m;
            try { m = config.getClass().getMethod("getAlwaysOpenPercentage"); }
            catch (NoSuchMethodException e) { m = config.getClass().getMethod("getAlwaysOpenPercent"); }
            return (double) m.invoke(config);
        } catch (Exception ignored) { return 1.0; }
    }

    private double getDeployAltitudeThreshold() {
        try {
            Method m = config.getClass().getMethod("getDeployAltitudeThreshold");
            return (double) m.invoke(config);
        } catch (Exception ignored) { return 0.0; }
    }

    private double getMaxMachForDeployment() {
        try {
            Method m = config.getClass().getMethod("getMaxMachForDeployment");
            return (double) m.invoke(config);
        } catch (Exception ignored) { return Double.MAX_VALUE; }
    }

    private double getTargetApogee() {
        try {
            Method m = config.getClass().getMethod("getTargetApogee");
            return (double) m.invoke(config);
        } catch (Exception ignored) { return 0.0; }
    }

    private double getApogeeToleranceMeters() {
        try {
            Method mOpt = config.getClass().getMethod("getApogeeToleranceMeters");
            Object v = mOpt.invoke(config);
            if (v instanceof Optional<?>) {
                Optional<?> o = (Optional<?>) v;
                if (o.isPresent() && o.get() instanceof Double) return (Double) o.get();
                return DEFAULT_TOLERANCE;
            }
            if (v instanceof Double) return (Double) v;
        } catch (Exception ignored) { }
        return DEFAULT_TOLERANCE;
    }

    // ───────────────────────────────────────────────────────────────────
    // Burn/Coast gating – robust to OR API differences
    // ───────────────────────────────────────────────────────────────────
    private boolean isMotorBurning(SimulationStatus status) {
        // 1) Preferred: explicit API (when available)
        try {
            Method m = status.getClass().getMethod("isMotorBurning");
            return (boolean) m.invoke(status);
        } catch (Exception ignored) { /* fall through */ }

        // 2) Fallback: thrust > ~0.5 N from FlightData (if available)
        try {
            double thrust = status.getFlightData().getLast(FlightDataType.TYPE_THRUST_FORCE);
            return Double.isFinite(thrust) && thrust > 0.5;
        } catch (Throwable ignored) { }

        // 3) Unknown → assume not burning
        return false;
    }

    // ───────────────────────────────────────────────────────────────────
    // Utilities
    // ───────────────────────────────────────────────────────────────────
    private static double clamp01(double v) {
        return (v < 0) ? 0 : (v > 1 ? 1 : v);
    }

    private static double gravity(double altitude) {
        final double r = EARTH_RADIUS + Math.max(0.0, altitude);
        return G0 * (EARTH_RADIUS * EARTH_RADIUS) / (r * r);
    }

    private static Double safeD(double v) {
        return Double.isFinite(v) ? v : null;
    }

    // Read OR's current apogee (best-effort; varies by build)
    private static double getOpenRocketApogee(final SimulationStatus st) {
        if (st == null || st.getFlightData() == null) return Double.NaN;
        final String[] candidates = new String[] {
                "TYPE_APOGEE",
                "TYPE_MAX_ALTITUDE",
                "TYPE_APOGEE_ESTIMATE",
                "TYPE_APOGEE_PREDICTION"
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
                double val = st.getFlightData().getLast((FlightDataType) ft);
                return Double.isFinite(val) ? val : null;
            }
        } catch (Throwable ignored) { }
        return null;
    }

    // ==================================================================
    //                    Internal realtime predictor
    // ==================================================================
    private final class ApogeePredictor {

        // Fit model: a(t) = A (1 − B t)^4
        private final ParametricUnivariateFunction CURVE = new ParametricUnivariateFunction() {
            @Override public double value(double t, double... p) {
                final double A = p[0], B = p[1];
                final double s = 1.0 - B * t;
                return A * Math.pow(s, 4);
            }
            @Override public double[] gradient(double t, double... p) {
                final double A = p[0], B = p[1];
                final double s = 1.0 - B * t;
                return new double[]{ Math.pow(s, 4), -4.0 * A * t * Math.pow(s, 3) };
            }
        };

        private final List<Double> accels = new ArrayList<>();
        private final List<Double> dts    = new ArrayList<>();
        private double currentAlt = 0.0;
        private double currentVel = 0.0;

        private boolean hasConverged = false;
        private double[] coeffs = { 0.0, 0.0 };
        private double lastRmse = Double.NaN;      // <- Option B: primitive with NaN

        private List<Double> lutVel    = Collections.emptyList(); // ascending v
        private List<Double> lutDeltaH = Collections.emptyList(); // Δh to apex

        private String lastMode = "BALLISTIC"; // "FIT" when LUT path used
        private double lastPredictedApogee = Double.NaN;

        void reset() {
            accels.clear();
            dts.clear();
            lutVel = Collections.emptyList();
            lutDeltaH = Collections.emptyList();
            hasConverged = false;
            coeffs[0] = coeffs[1] = 0.0;
            lastRmse = Double.NaN;            // <- never null
            lastMode = "BALLISTIC";
            lastPredictedApogee = Double.NaN;

            if (debugEnabled && log.isDebugEnabled()) {
                log.debug("[APOGEE] predictor reset");
            }
        }

        void update(double accelNet, double dt, double altitude, double velocity) {
            currentAlt = altitude;
            currentVel = velocity;
            if (!Double.isFinite(dt) || dt <= 1e-6) return;

            if (Double.isFinite(accelNet)) {
                accels.add(accelNet);
                dts.add(dt);
            }

            if (!hasConverged && accels.size() >= MIN_PACKETS) {
                fitCurve();
            }
        }

        double getPredictedApogee() {
            if (hasConverged && !lutVel.isEmpty()) {
                lastMode = "FIT";
                lastPredictedApogee = currentAlt + interpDeltaH(currentVel);
                return lastPredictedApogee;
            }
            // Fallback: ballistic (upper bound). Use local g(h).
            lastMode = "BALLISTIC";
            final double g = gravity(currentAlt);
            lastPredictedApogee = currentAlt + (currentVel * currentVel) / (2.0 * g);
            return lastPredictedApogee;
        }

        String   getLastMode()     { return lastMode; }
        boolean  hasConverged()    { return hasConverged; }
        double   getLastRmse()     { return lastRmse; }            // <- primitive
        double[] getLastCoeffs()   { return coeffs != null ? coeffs.clone() : null; }

        // ------------------------ fitting & LUT ------------------------
        private void fitCurve() {
            final int n = accels.size();
            double[] t = new double[n];
            double cum = 0.0;
            for (int i = 0; i < n; i++) { cum += dts.get(i); t[i] = cum; }

            List<WeightedObservedPoint> obs = new ArrayList<>(n);
            for (int i = 0; i < n; i++) {
                obs.add(new WeightedObservedPoint(1.0, t[i], accels.get(i)));
            }

            // Initial guess: near measured mean accel and gentle decay
            final double meanA = accels.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
            double[] guess = new double[]{ meanA, 0.02 };

            try {
                coeffs = SimpleCurveFitter.create(CURVE, guess).fit(obs);
                lastRmse = computeRMSE(obs, coeffs);
                hasConverged = (lastRmse < MAX_RMSE_FOR_CONV) &&
                               Double.isFinite(coeffs[0]) && Double.isFinite(coeffs[1]);

                if (debugEnabled && log.isDebugEnabled()) {
                    log.debug("[APOGEE] fit A={} B={} rmse={} n={}",
                            safeD(coeffs[0]), safeD(coeffs[1]),
                            Double.isFinite(lastRmse) ? lastRmse : "n/a", n);
                }

                if (hasConverged) buildLookupTable();
            } catch (Exception e) {
                hasConverged = false;
                lastRmse = Double.NaN;
                if (debugEnabled && log.isDebugEnabled()) {
                    log.debug("[APOGEE] fit failed: {}", e.toString());
                }
            }
        }

        private double computeRMSE(List<WeightedObservedPoint> obs, double[] p) {
            double sum = 0.0;
            for (WeightedObservedPoint o : obs) {
                double e = o.getY() - CURVE.value(o.getX(), p);
                sum += e * e;
            }
            return Math.sqrt(sum / Math.max(1, obs.size()));
        }

        private void buildLookupTable() {
            List<Double> vOut = new ArrayList<>();
            List<Double> hOut = new ArrayList<>();

            double v = Math.max(0.0, currentVel);
            double h = 0.0;
            double time = 0.0;
            final double dt = 0.02; // internal integrator step

            // Integrate forward until v crosses zero or a reasonable cap
            while (v > 0.0 && time < 40.0) {
                double aFit = CURVE.value(time, coeffs);
                final double aNet = FIT_IS_NET_ACCEL ? aFit : (aFit - gravity(currentAlt));
                v += aNet * dt;
                if (v < 0.0) break;
                h += v * dt;
                vOut.add(v);
                hOut.add(h);
                time += dt;
            }

            if (vOut.isEmpty()) {
                lutVel = Collections.emptyList();
                lutDeltaH = Collections.emptyList();
                hasConverged = false;
                if (debugEnabled && log.isDebugEnabled()) {
                    log.debug("[APOGEE] LUT build produced no points; back to ballistic");
                }
                return;
            }

            // Interpolation expects ascending velocity
            Collections.reverse(vOut);
            Collections.reverse(hOut);
            lutVel = vOut;
            lutDeltaH = hOut;

            if (debugEnabled && log.isDebugEnabled()) {
                log.debug("[APOGEE] LUT built: points={}", lutVel.size());
            }
        }

        private double interpDeltaH(double velocity) {
            if (lutVel.isEmpty()) return 0.0;
            // clamp
            if (velocity >= lutVel.get(lutVel.size()-1)) return lutDeltaH.get(lutDeltaH.size()-1);
            if (velocity <= lutVel.get(0))              return lutDeltaH.get(0);

            // find bracket
            for (int i = 0; i < lutVel.size() - 1; i++) {
                double v1 = lutVel.get(i);
                double v2 = lutVel.get(i + 1);
                if (velocity >= v1 && velocity <= v2) {
                    double h1 = lutDeltaH.get(i);
                    double h2 = lutDeltaH.get(i + 1);
                    double f  = (velocity - v1) / (v2 - v1);
                    return h1 + f * (h2 - h1);
                }
            }
            return lutDeltaH.get(lutDeltaH.size()-1);
        }
    }
}
