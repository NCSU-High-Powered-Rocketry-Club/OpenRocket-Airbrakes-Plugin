package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * AirbrakeController — minimal bang-bang:
 *   if (apogee > target && !extended) -> extend
 *   else if (apogee <= target && extended) -> retract + switch_altitude_back_to_pressure
 *   else hold
 *
 * Public API kept compatible.
 */
public final class AirbrakeController {

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        default void switch_altitude_back_to_pressure() {}
    }

    public interface DebugListener {
        void addController(double timeSeconds, double u, String reason);
    }

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    private final double targetApogeeMeters;
    private final ApogeePredictor predictor;
    private ControlContext context;
    private DebugListener dbg;

    // Kept for API compatibility (unused)
    private final double minCoastSeconds;
    private final double maxCoastSeconds;
    private double deadband;

    private Boolean airbrakesExtended = null;  // null=unknown; true=extended; false=retracted
    private boolean hasSetpoint = false;
    private double currentSetpointU = 0.0;     // 0 or 1

    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context) {
        this(targetApogeeMeters, predictor, context, 0.0, 0.0, 0.0);
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

    public void setContext(ControlContext ctx) { this.context = ctx; }
    public void setDebugListener(DebugListener dbg) { this.dbg = dbg; }
    public double getTargetApogeeMeters() { return targetApogeeMeters; }
    public double getDeadbandMeters() { return deadband; }
    public void setApogeeDeadbandMeters(double meters) { this.deadband = Math.max(0.0, meters); }
    public void setMinFlipIntervalSec(double s) { /* no-op */ }

    public boolean hasSetpoint() { return hasSetpoint; }
    public double currentSetpoint() { return currentSetpointU; }
    public Boolean lastExtendedState() { return airbrakesExtended; }

    public void reset() {
        airbrakesExtended = null;
        hasSetpoint = false;
        currentSetpointU = 0.0;
    }

    public void notifyCoastLatched(double tSeconds) { /* no-op */ }
    public void notifyCoastLatched(SimulationStatus status) { /* no-op */ }

    public boolean updateAndGateFlexible(SimulationStatus status) {
        if (predictor == null) return false;

        // Prefer STRICT, else BEST-EFFORT
        Double ap = predictor.getPredictionIfReady();
        if (ap == null) ap = predictor.getApogeeBestEffort();
        if (ap == null || !Double.isFinite(ap)) {
            log.debug("[Controller] t={}s: no apogee estimate yet — holding", fmt(safeTime(status)));
            return false;
        }

        boolean changed = false;
        if (ap > targetApogeeMeters && !Boolean.TRUE.equals(airbrakesExtended)) {
            airbrakesExtended = true;
            currentSetpointU = 1.0; hasSetpoint = true;
            if (context != null) context.extend_airbrakes();
            changed = true;
            if (dbg != null) dbg.addController(safeTime(status), 1.0, "extend: apogee>target");
            log.info("[Controller] apogee={} > target={} → EXTEND", fmt(ap), fmt(targetApogeeMeters));
        } else if (ap <= targetApogeeMeters && Boolean.TRUE.equals(airbrakesExtended)) {
            airbrakesExtended = false;
            currentSetpointU = 0.0; hasSetpoint = true;
            if (context != null) {
                context.retract_airbrakes();
                context.switch_altitude_back_to_pressure();
            }
            changed = true;
            if (dbg != null) dbg.addController(safeTime(status), 0.0, "retract: apogee<=target");
            log.info("[Controller] apogee={} ≤ target={} → RETRACT", fmt(ap), fmt(targetApogeeMeters));
        } else {
            if (airbrakesExtended != null) { hasSetpoint = true; currentSetpointU = airbrakesExtended ? 1.0 : 0.0; }
            log.debug("[Controller] apogee={} vs target={} — hold", fmt(ap), fmt(targetApogeeMeters));
        }
        return changed;
    }

    private static double safeTime(SimulationStatus status) {
        try { double t = status.getSimulationTime(); return Double.isFinite(t) ? t : 0.0; }
        catch (Throwable t) { return 0.0; }
    }
    private static String fmt(double x) { return String.format("%.3f", x); }
}
