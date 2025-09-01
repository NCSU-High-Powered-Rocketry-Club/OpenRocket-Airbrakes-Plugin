package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * AirbrakeController
 *
 * Flexible gate:
 *  - If predictor has a STRICT apogee (converged), use it.
 *  - Else, once a minimum coast window has elapsed AND a best-effort value exists, use BEST-EFFORT.
 *  - Otherwise, do nothing (return false).
 *
 * Commands:
 *  - extend_airbrakes() when predicted apogee > target
 *  - retract_airbrakes() otherwise
 *
 * Returns:
 *  - updateAndGateFlexible(...) returns true ONLY when a new command is issued on this call.
 */
public final class AirbrakeController {

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        void switch_altitude_back_to_pressure();
    }

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    // --- Config ---
    private final double targetApogeeMeters;
    private final ApogeePredictor predictor;
    private ControlContext context;

    // Coast gating windows (seconds)
    private final double minCoastSeconds;
    private final double maxCoastSeconds; // not strictly required, but kept for compatibility/telemetry

    // Optional deadband (m) around target for stability; keep small or zero if you prefer bang-bang
    private double apogeeDeadbandMeters = 0.0;

    // --- Internal state ---
    private Double firstCoastTime = null; // wall time when predictor received first coast sample
    private Boolean lastExtended = null;  // null = no command yet; true=extended; false=retracted

    // ----------------- Constructors -----------------

    /** Backward-compatible ctor (no explicit windows). */
    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context) {
        this(targetApogeeMeters, predictor, context, 2.0, 5.0);
    }

    /** Preferred ctor with coast windows. */
    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context,
                              double minCoastSeconds,
                              double maxCoastSeconds) {
        this.targetApogeeMeters = targetApogeeMeters;
        this.predictor = predictor;
        this.context = context;
        this.minCoastSeconds = Math.max(0.0, minCoastSeconds);
        this.maxCoastSeconds = Math.max(this.minCoastSeconds, maxCoastSeconds);
    }

    // ----------------- API -----------------

    public void setContext(ControlContext ctx) { this.context = ctx; }

    public double getTargetApogeeMeters() { return targetApogeeMeters; }

    /** Optional tweak: adjust the deadband (+/−) around target before commanding. */
    public void setApogeeDeadbandMeters(double meters) {
        this.apogeeDeadbandMeters = Math.max(0.0, meters);
    }

    /**
     * Decide and (if needed) issue a new command.
     * @return true if a new command was issued in THIS call, false otherwise.
     */
    public boolean updateAndGateFlexible(SimulationStatus status) {
        if (predictor == null) return false;

        final double t = safeTime(status);

        // Initialize coast reference time the first time we see predictor samples.
        if (firstCoastTime == null && predictor.sampleCount() > 0) {
            firstCoastTime = t;
            log.debug("[Controller] t={}s: first coast sample observed; coast window begins",
                    fmt(t));
        }

        final double coastElapsed = (firstCoastTime == null) ? 0.0 : Math.max(0.0, t - firstCoastTime);

        // Strict (converged) prediction if available
        final Double apStrict = predictor.getPredictionIfReady();

        // Best-effort allowed only after min window
        Double apBestEffort = null;
        if (apStrict == null && coastElapsed >= minCoastSeconds) {
            apBestEffort = predictor.getApogeeBestEffort();
        }

        final boolean usingStrict = (apStrict != null);
        final Double ap = usingStrict ? apStrict : apBestEffort;

        if (ap == null) {
            // Not ready to act
            log.debug("[Controller] t={}s: no usable apogee yet (coastElapsed={}s, min={}s) — holding",
                    fmt(t), fmt(coastElapsed), fmt(minCoastSeconds));
            return false;
        }

        // Decide: extend when predicted apogee is meaningfully ABOVE target
        final double err = ap - targetApogeeMeters;
        final boolean shouldExtend = (err > apogeeDeadbandMeters);

        // Issue command only if different from last state
        if (lastExtended == null || lastExtended != shouldExtend) {
            lastExtended = shouldExtend;
            if (context != null) {
                if (shouldExtend) {
                    context.extend_airbrakes();
                    log.info("[Controller] t={}s: using {} apogee={} -> EXTEND (target={}, err={})",
                            fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT", fmt(ap),
                            fmt(targetApogeeMeters), fmt(err));
                } else {
                    context.retract_airbrakes();
                    log.info("[Controller] t={}s: using {} apogee={} -> RETRACT (target={}, err={})",
                            fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT", fmt(ap),
                            fmt(targetApogeeMeters), fmt(err));
                }
                // (Optional) context.switch_altitude_back_to_pressure() — left to caller as needed
            }
            return true; // acted this call
        }

        // Command unchanged this tick
        log.debug("[Controller] t={}s: using {} apogee={} (target={}, err={}) — no change",
                fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT", fmt(ap),
                fmt(targetApogeeMeters), fmt(err));
        return false;
    }

    // ----------------- Helpers -----------------

    private static double safeTime(SimulationStatus status) {
        try {
            double t = status.getSimulationTime();
            return Double.isFinite(t) ? t : 0.0;
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private static String fmt(double x) { return String.format("%.3f", x); }
}
