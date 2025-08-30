package com.airbrakesplugin;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;

public final class AirbrakeController {

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        void switch_altitude_back_to_pressure();
    }

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    private final double targetApogeeMeters;
    private final ApogeePredictor predictor;
    private ControlContext context;  // Changed from final to allow setting in tests

    private final double minCoastSeconds;
    private final double maxCoastSeconds;

    private boolean airbrakesExtended = false;
    private boolean coastStarted = false;
    private double coastStartSimTime = Double.NaN;

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
    }

    /**
     * Sets a new control context.
     * This method is primarily used for testing.
     * 
     * @param newContext The new control context to use
     */
    public void setContext(ControlContext newContext) {
        this.context = newContext;
    }

    /** Returns true if a control action was taken; false if still waiting. */
    public boolean updateAndGateFlexible(final SimulationStatus status) {
        final double now = status.getSimulationTime();

        // Coast window bookkeeping: start when predictor first has any sample
        if (!coastStarted && predictor.sampleCount() > 0) {
            coastStarted = true;
            coastStartSimTime = now;
        }
        final double coastSeconds = coastStarted ? Math.max(0.0, now - coastStartSimTime) : 0.0;

        // STRICT path
        if (predictor.hasConverged() && coastSeconds >= minCoastSeconds) {
            final Double apogee = predictor.getPredictionIfReady();
            if (apogee != null && Double.isFinite(apogee)) {
                log.debug("[Airbrakes] controller: using STRICT apogee {}", String.format("%.2f", apogee));
                act(apogee);
                return true;
            }
            return false;
        }

        // BEST-EFFORT path
        if (coastSeconds >= minCoastSeconds && coastSeconds <= maxCoastSeconds) {
            final Double apogeeBE = predictor.getApogeeBestEffort();
            if (apogeeBE != null && Double.isFinite(apogeeBE)) {
                log.debug("[Airbrakes] controller: using BEST-EFFORT apogee {}", String.format("%.2f", apogeeBE));
                act(apogeeBE);
                return true;
            }
        }
        return false;
    }

    private void act(double apogee) {
        if (apogee > targetApogeeMeters && !airbrakesExtended) {
            if (context != null) context.extend_airbrakes();
            airbrakesExtended = true;
        } else if (apogee <= targetApogeeMeters && airbrakesExtended) {
            if (context != null) {
                context.retract_airbrakes();
                context.switch_altitude_back_to_pressure();
            }
            airbrakesExtended = false;
        }
    }

    public boolean isAirbrakesExtended() { return airbrakesExtended; }
    public double getTargetApogeeMeters() { return targetApogeeMeters; }
}
