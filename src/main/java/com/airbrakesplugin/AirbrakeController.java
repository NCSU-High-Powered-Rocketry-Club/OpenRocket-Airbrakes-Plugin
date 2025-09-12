package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * AirbrakeController — bang-bang with symmetric hysteresis, strict-first gating,
 * and a persistent "current setpoint" the listener can poll every tick.
 *
 * Policy:
 *  - STRICT (converged) apogee wins whenever available.
 *  - Otherwise, after minCoastSeconds since COAST START, use BEST-EFFORT.
 *  - If neither is usable, hold (no update).
 *
 * Hysteresis (deadband = ±db around target):
 *  - First decision:  EXTEND only if err > +db, else RETRACT.
 *  - EXTENDED   ->    RETRACT only if err < -db.
 *  - RETRACTED  ->    EXTEND  as soon as err ≥ 0   (earlier re-extend to cut overshoot).
 *
 * Returns: updateAndGateFlexible(...) is true only if a NEW edge was issued this call.
 * Additionally, caller can read currentSetpoint() every tick to drive the actuator.
 */
public final class AirbrakeController {

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        default void switch_altitude_back_to_pressure() {} // optional; noop by default
    }

    public interface DebugListener {
        void addController(double timeSeconds, double u, String reason);
    }

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    // ---- Config ----
    private final double targetApogeeMeters;
    private final ApogeePredictor predictor;
    private ControlContext context;
    private DebugListener dbg;

    private final double minCoastSeconds;
    private final double maxCoastSeconds;   // kept for logging/compatibility
    private double deadband;                // meters

    // Optional: prevent chatter from rapid target crossings
    private double minFlipIntervalSec = 0.0;

    // ---- State ----
    private Double coastStartTime = null;   // COAST start anchor
    private Boolean lastExtended   = null;  // null=unknown; true=extended; false=retracted
    private double lastFlipTimeSec = Double.NEGATIVE_INFINITY;

    // Persistent setpoint (what the listener should head toward each tick)
    private boolean hasSetpoint = false;
    private double currentSetpointU = 0.0;  // 0=retract, 1=extend

    // ----------------- Constructors -----------------
    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context) {
        this(targetApogeeMeters, predictor, context, 2.0, 5.0, 0.0);
    }

    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context,
                              double minCoastSeconds,
                              double maxCoastSeconds) {
        this(targetApogeeMeters, predictor, context, minCoastSeconds, maxCoastSeconds, 0.0);
    }

    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context,
                              double minCoastSeconds,
                              double maxCoastSeconds,
                              double deadbandMeters) {
        this.targetApogeeMeters = targetApogeeMeters;
        this.predictor = predictor;
        this.context = context;
        this.minCoastSeconds = Math.max(0.0, minCoastSeconds);
        this.maxCoastSeconds = Math.max(this.minCoastSeconds, maxCoastSeconds);
        this.deadband = Math.max(0.0, deadbandMeters);
    }

    // ----------------- API -----------------
    public void setContext(ControlContext ctx) { this.context = ctx; }
    public void setDebugListener(DebugListener dbg) { this.dbg = dbg; }
    public double getTargetApogeeMeters() { return targetApogeeMeters; }
    public double getDeadbandMeters() { return deadband; }
    public void setApogeeDeadbandMeters(double meters) { this.deadband = Math.max(0.0, meters); }
    public void setMinFlipIntervalSec(double s) { this.minFlipIntervalSec = Math.max(0.0, s); }

    /** Persistent setpoint (0=retract, 1=extend). Valid once we've made a first decision. */
    public boolean hasSetpoint() { return hasSetpoint; }
    public double currentSetpoint() { return currentSetpointU; }
    public Boolean lastExtendedState() { return lastExtended; }

    public void reset() {
        coastStartTime = null;
        lastExtended = null;
        lastFlipTimeSec = Double.NEGATIVE_INFINITY;
        hasSetpoint = false;
        currentSetpointU = 0.0;
    }

    /** Prefer the listener to call this exactly at coast start. */
    public void notifyCoastLatched(double tSeconds) {
        if (coastStartTime == null) {
            coastStartTime = tSeconds;
            log.debug("[Controller] t={}s: COAST start latched (anchor set)", fmt(tSeconds));
        }
    }
    public void notifyCoastLatched(SimulationStatus status) { notifyCoastLatched(safeTime(status)); }

    /**
     * Decide and (if needed) issue a new command.
     * @return true if a NEW edge (extend or retract) was issued in THIS call.
     */
    public boolean updateAndGateFlexible(SimulationStatus status) {
        if (predictor == null) return false;

        final double t = safeTime(status);

        // If no explicit anchor, infer coast when first samples arrive (fallback).
        if (coastStartTime == null && predictor.sampleCount() > 0) {
            coastStartTime = t;
            log.debug("[Controller] t={}s: inferred COAST anchor from first samples", fmt(t));
        }
        final double elapsed = (coastStartTime == null) ? 0.0 : Math.max(0.0, t - coastStartTime);

        // STRICT has precedence; otherwise gate BEST-EFFORT on min coast time.
        final Double apStrict = predictor.getPredictionIfReady();
        final Double ap = (apStrict != null)
                ? apStrict
                : (elapsed >= minCoastSeconds ? predictor.getApogeeBestEffort() : null);

        if (ap == null || !Double.isFinite(ap)) {
            if (coastStartTime == null) {
                log.debug("[Controller] t={}s: no usable apogee (no coast anchor yet) — holding", fmt(t));
            } else if (elapsed < minCoastSeconds) {
                log.debug("[Controller] t={}s: gating BEST-EFFORT (elapsed={}s < min={}s) — holding",
                        fmt(t), fmt(elapsed), fmt(minCoastSeconds));
            } else {
                log.debug("[Controller] t={}s: >={}s since coast; still no usable apogee — holding",
                        fmt(t), fmt(minCoastSeconds));
            }
            // Do not change setpoint here; listener keeps ramping toward whatever it last knew.
            return false;
        }

        // Error relative to target
        final double err = ap - targetApogeeMeters;

        // Decide desired "extended?" using improved hysteresis
        boolean wantExtended = decideNext(t, lastExtended, err, deadband, minFlipIntervalSec, lastFlipTimeSec);

        // Update persistent setpoint (what the listener will head toward every tick)
        hasSetpoint = true;
        currentSetpointU = wantExtended ? 1.0 : 0.0;

        // Optional debug hook for plotting/controller UI
        if (dbg != null) {
            double u; String why;
            if      (err >  deadband) { u = 1.0; why = "over-target"; }
            else if (err < -deadband) { u = 0.0; why = "under-target"; }
            else if (wantExtended)    { u = 1.0; why = "deadband-hold-EXT"; }
            else                      { u = 0.0; why = "deadband-hold-RET"; }
            dbg.addController(t, u, why);
        }

        // Emit edge only if state changes
        if (lastExtended == null || !lastExtended.equals(wantExtended)) {
            lastExtended = wantExtended;
            lastFlipTimeSec = t;
            if (context != null) {
                if (wantExtended) {
                    context.extend_airbrakes();
                    log.info("[Controller] t={}s: apogee={} -> EXTEND (target={}, err={}, db=±{})",
                            fmt(t), fmt(ap), fmt(targetApogeeMeters), fmt(err), fmt(deadband));
                } else {
                    context.retract_airbrakes();
                    log.info("[Controller] t={}s: apogee={} -> RETRACT (target={}, err={}, db=±{})",
                            fmt(t), fmt(ap), fmt(targetApogeeMeters), fmt(err), fmt(deadband));
                }
            }
            return true;
        }

        log.debug("[Controller] t={}s: apogee={} (target={}, err={}, db=±{}) — no change",
                fmt(t), fmt(ap), fmt(targetApogeeMeters), fmt(err), fmt(deadband));
        return false;
    }

    // ----------------- Helpers -----------------

    /** Improved hysteresis: early re-extend on target crossing; optional dwell between flips. */
    private static boolean decideNext(double tNow,
                                      Boolean lastExtended,
                                      double err,
                                      double db,
                                      double minFlipInterval,
                                      double lastFlipTimeSec) {
        // Honor dwell window if requested
        if (minFlipInterval > 0.0 && lastFlipTimeSec > 0.0) {
            double since = tNow - lastFlipTimeSec;
            if (since < minFlipInterval && lastExtended != null) {
                return lastExtended;
            }
        }

        if (lastExtended == null) {
            // First decision: conservative — require clear overshoot to start extending
            return err > db;
        }
        if (lastExtended) {
            // Currently EXTENDED: only retract if clearly below the band
            return !(err < -db);
        }
        // Currently RETRACTED: re-extend promptly once we cross the target
        return (err >= 0.0);
    }

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
