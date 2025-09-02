package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * AirbrakeController (bang-bang + hysteresis + flexible gating)
 *
 * Gate:
 *  - STRICT (converged) prediction always takes precedence if available.
 *  - Otherwise, after a minimum coast window has elapsed (latched when the first
 *    BEST-EFFORT apogee appears), act on BEST-EFFORT.
 *  - If neither is available, do nothing.
 *
 * Commands:
 *  - EXTEND when predicted apogee above target (with hysteresis).
 *  - RETRACT when predicted apogee below target (with hysteresis).
 *
 * Deadband:
 *  - Uses symmetric hysteresis: retracted→extend only if err > +DB; extended→retract only if err < −DB.
 *
 * Return value:
 *  - updateAndGateFlexible(...) returns true ONLY when a new command is issued on this call.
 */
public final class AirbrakeController {

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        void switch_altitude_back_to_pressure(); // optional, left to caller
    }

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    // --- Config ---
    private final double targetApogeeMeters;
    private final ApogeePredictor predictor;
    private ControlContext context;

    // Coast gating windows (seconds)
    private final double minCoastSeconds;
    private final double maxCoastSeconds; // used for diagnostics; not a hard gate

    // Symmetric deadband half-width around target (meters) for hysteresis
    private double apogeeDeadbandMeters = 0.0;

    // --- Internal state ---
    private Double gateStartTime = null;   // when BEST-EFFORT first became available
    private Boolean lastExtended = null;   // null=no command yet; true=extended; false=retracted
    private Boolean lastUsingStrict = null;

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

    /** Optional tweak: adjust the symmetric deadband (+/−meters) around target. */
    public void setApogeeDeadbandMeters(double meters) {
        this.apogeeDeadbandMeters = Math.max(0.0, meters);
    }

    /** Call at simulation start to clear internal latches between runs. */
    public void reset() {
        gateStartTime = null;
        lastExtended = null;
        lastUsingStrict = null;
    }

    /**
     * Decide and (if needed) issue a new command.
     * @return true if a new command was issued in THIS call, false otherwise.
     */
    public boolean updateAndGateFlexible(SimulationStatus status) {
        if (predictor == null) return false;

        final double t = safeTime(status);

        // Latch gate start when a best-effort apogee FIRST becomes available.
        // (This aligns the "minCoastSeconds" window with the earliest usable estimate.)
        if (gateStartTime == null) {
            final Double probe = predictor.getApogeeBestEffort();
            if (probe != null && Double.isFinite(probe)) {
                gateStartTime = t;
                log.debug("[Controller] t={}s: best-effort apogee appeared; gate window starts", fmt(t));
            }
            // If you prefer the old behavior based on predictor.sampleCount():
            // if (gateStartTime == null && predictor.sampleCount() > 0) { gateStartTime = t; ... }
        }

        final double elapsed = (gateStartTime == null) ? 0.0 : Math.max(0.0, t - gateStartTime);

        // STRICT (converged) prediction if available
        final Double apStrict = predictor.getPredictionIfReady();

        // BEST-EFFORT allowed only after min window
        Double apBestEffort = null;
        if (apStrict == null && elapsed >= minCoastSeconds) {
            apBestEffort = predictor.getApogeeBestEffort();
        }

        final boolean usingStrict = (apStrict != null);
        final Double ap = usingStrict ? apStrict : apBestEffort;

        if (ap == null || !Double.isFinite(ap)) {
            // Not ready to act
            if (gateStartTime != null && elapsed > maxCoastSeconds) {
                log.debug("[Controller] t={}s: >{}s since gate start; still no STRICT. Holding on best-effort only when window permits.",
                        fmt(t), fmt(maxCoastSeconds));
            } else {
                log.debug("[Controller] t={}s: no usable apogee yet (elapsed={}s, min={}s) — holding",
                        fmt(t), fmt(elapsed), fmt(minCoastSeconds));
            }
            return false;
        }

        // Optional diagnostic if mode flips
        if (lastUsingStrict == null || lastUsingStrict != usingStrict) {
            log.debug("[Controller] t={}s: mode -> {}", fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT");
            lastUsingStrict = usingStrict;
        }

        // Decision with symmetric hysteresis around target
        final double err = ap - targetApogeeMeters;
        final double db = apogeeDeadbandMeters;

        boolean nextExtended;
        if (lastExtended == null) {
            // First decision: choose based on center + one-sided threshold (extend only if clearly above)
            nextExtended = (err > db);
        } else if (lastExtended) {
            // Currently EXTENDED: only retract if we are clearly below target band
            nextExtended = !(err < -db);
        } else {
            // Currently RETRACTED: only extend if we are clearly above target band
            nextExtended = (err > db);
        }

        // Only issue a command if the state changes
        if (lastExtended == null || lastExtended != nextExtended) {
            lastExtended = nextExtended;
            if (context != null) {
                if (nextExtended) {
                    context.extend_airbrakes();
                    log.info("[Controller] t={}s: {} apogee={} -> EXTEND (target={}, err={}, db=±{})",
                            fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT", fmt(ap),
                            fmt(targetApogeeMeters), fmt(err), fmt(db));
                } else {
                    context.retract_airbrakes();
                    log.info("[Controller] t={}s: {} apogee={} -> RETRACT (target={}, err={}, db=±{})",
                            fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT", fmt(ap),
                            fmt(targetApogeeMeters), fmt(err), fmt(db));
                }
                // Optional: context.switch_altitude_back_to_pressure();
            }
            return true; // acted this call
        }

        // Command unchanged this tick
        log.debug("[Controller] t={}s: {} apogee={} (target={}, err={}, db=±{}) — no change",
                fmt(t), usingStrict ? "STRICT" : "BEST-EFFORT", fmt(ap),
                fmt(targetApogeeMeters), fmt(err), fmt(db));
        return false;
    }

    // ----------------- Helpers -----------------

    private static double safeTime(SimulationStatus status) {
        try {
            final double t = status.getSimulationTime();
            return Double.isFinite(t) ? t : 0.0;
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private static String fmt(double x) { return String.format("%.3f", x); }
}
