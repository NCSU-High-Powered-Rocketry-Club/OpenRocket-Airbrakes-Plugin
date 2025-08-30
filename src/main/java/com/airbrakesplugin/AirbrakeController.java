package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Bang-bang controller with a HARD GATE:
 * - The simulation's control loop will not proceed (from this controller) until
 *   the apogee predictor has converged using >= minCoastSeconds of coast data.
 *
 * Usage:
 *   if (!controller.updateAndGate(status)) {
 *       // hold this step (no actuator changes); listener can simply return true
 *       // to keep sim running forward to gather more coast samples.
 *   }
 */
public final class AirbrakeController {
    private static final Logger logger = LoggerFactory.getLogger(AirbrakeController.class);

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        void switch_altitude_back_to_pressure();
    }

    private final double targetApogeeMeters;
    private final ApogeePredictor predictor;
    private final ControlContext context;

    // --- gating knobs ---
    private final double minCoastSeconds; // >= 2s as requested
    private final double maxCoastSeconds; // optional upper bound (not strictly required)

    // --- internal state for the gate ---
    private boolean airbrakesExtended = false;
    private boolean coastStarted = false;
    private double coastStartSimTime = Double.NaN;
    private int    lastSampleCount   = 0;

    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictor predictor,
                              ControlContext context) {
        this(targetApogeeMeters, predictor, context, 2.0, 5.0);
    }

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
        
        logger.debug("AirbrakeController initialized: target={}m, minCoast={}s, maxCoast={}s", 
                     targetApogeeMeters, this.minCoastSeconds, this.maxCoastSeconds);
    }

    /**
     * Update with HARD GATE. Returns true if control was executed this step,
     * false if the step is being held waiting for convergence.
     */
    public boolean updateAndGate(final SimulationStatus status) {
        final double now = status.getSimulationTime();
        logger.debug("updateAndGate: t={}s", now);

        // Detect first time the predictor received any coast sample this run.
        final int sc = predictor.sampleCount();
        if (!coastStarted && sc > 0) {
            coastStarted = true;
            coastStartSimTime = now;
            logger.debug("Coast phase detected: starting at t={}s", coastStartSimTime);
        }

        // Compute coast duration so far.
        final double coastSeconds = (coastStarted ? Math.max(0.0, now - coastStartSimTime) : 0.0);
        
        if (sc > lastSampleCount) {
            logger.debug("Samples: {} -> {} (coast: {:.2f}s)", lastSampleCount, sc, coastSeconds);
            lastSampleCount = sc;
        }

        // Gate condition: need predictor convergence AND >= minCoastSeconds of data.
        final boolean hasConverged = predictor.hasConverged();
        final boolean hasMinWindow = (coastSeconds >= minCoastSeconds);

        if (!(hasConverged && hasMinWindow)) {
            logger.debug("Gate not passed: hasConverged={}, coastSeconds={:.2f}, minRequired={}s", 
                         hasConverged, coastSeconds, minCoastSeconds);
            // Best-effort: try to pause/hold the sim if API offers it (reflection).
            requestPauseIfAvailable(status);
            return false; // tell caller to hold this step (no actuator changes)
        }

        logger.debug("Gate passed: hasConverged={}, coastSeconds={:.2f}s", hasConverged, coastSeconds);

        // At this point we are allowed to control.
        final Double apogeeObj = predictor.getPredictionIfReady();
        if (apogeeObj == null || !Double.isFinite(apogeeObj)) {
            // Converged but no numeric apogee? Hold this step anyway.
            logger.warn("Predictor converged but returned invalid apogee: {}", apogeeObj);
            requestPauseIfAvailable(status);
            return false;
        }

        final double apogee = apogeeObj;
        logger.debug("Predicted apogee: {:.2f}m (target: {:.2f}m)", apogee, targetApogeeMeters);

        // --- Simple bang-bang exactly as specified ---
        if (apogee > targetApogeeMeters && !airbrakesExtended) {
            logger.info("EXTENDING airbrakes: apogee {:.2f}m > target {:.2f}m", apogee, targetApogeeMeters);
            if (context != null) context.extend_airbrakes();
            airbrakesExtended = true;
        } else if (apogee <= targetApogeeMeters && airbrakesExtended) {
            logger.info("RETRACTING airbrakes: apogee {:.2f}m <= target {:.2f}m", apogee, targetApogeeMeters);
            if (context != null) {
                context.retract_airbrakes();
                context.switch_altitude_back_to_pressure();
            }
            airbrakesExtended = false;
        }

        return true;
    }

    // Legacy API (no gate). Keep if other code calls it; otherwise you can remove.
    public void update(final SimulationStatus status) {
        logger.debug("Legacy update() called, delegating to updateAndGate()");
        updateAndGate(status);
    }

    public boolean isAirbrakesExtended() { return airbrakesExtended; }
    public double getTargetApogeeMeters() { return targetApogeeMeters; }

    // --- best-effort pause hooks via reflection (no-ops if not present) ---
    private static void requestPauseIfAvailable(final SimulationStatus status) {
        logger.debug("Attempting to request pause via reflection");
        try {
            // Try common spellings observed across sim stacks.
            for (String m : new String[]{"requestPause", "setPaused", "pause", "requestHold", "requestStop"}) {
                try {
                    if ("setPaused".equals(m)) {
                        status.getClass().getMethod(m, boolean.class).invoke(status, true);
                        logger.debug("Pause requested via {}", m);
                        return;
                    } else {
                        status.getClass().getMethod(m).invoke(status);
                        logger.debug("Pause requested via {}", m);
                        return;
                    }
                } catch (NoSuchMethodException ignored) { /* try next */ }
            }
            logger.debug("No suitable pause method found");
        } catch (Throwable t) {
            logger.debug("Exception while requesting pause: {}", t.getMessage());
            // If nothing is available we silently continue — the listener will just "hold" this step.
        }
    }
}
