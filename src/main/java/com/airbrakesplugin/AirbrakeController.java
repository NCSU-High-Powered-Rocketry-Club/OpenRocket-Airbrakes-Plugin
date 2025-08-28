package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictorListener;
import info.openrocket.core.simulation.SimulationStatus;
import java.util.logging.Logger;
import java.util.logging.Level;

/**
 * Simple bang-bang controller using predictor's apogee (no convergence gate).
 */
public final class AirbrakeController {
    private static final Logger LOGGER = Logger.getLogger(AirbrakeController.class.getName());

    public interface ControlContext {
        void extend_airbrakes();
        void retract_airbrakes();
        void switch_altitude_back_to_pressure();
    }

    private final double targetApogeeMeters;
    private final ApogeePredictorListener predictor;
    private final ControlContext context;

    private boolean airbrakesExtended = false;

    public AirbrakeController(double targetApogeeMeters,
                              ApogeePredictorListener predictor,
                              ControlContext context) {
        this.targetApogeeMeters = targetApogeeMeters;
        this.predictor = predictor;
        this.context = context;
        LOGGER.info("AirbrakeController initialized with target apogee: " + targetApogeeMeters + " meters");
    }

    /** Call once per sim step. */
    public void update(final SimulationStatus status) {
        final double apogee = predictor.getLatestPredictedApogee();
        LOGGER.fine("Update called: predicted apogee=" + apogee + ", target=" + targetApogeeMeters + 
                   ", airbrakes extended=" + airbrakesExtended);
        
        if (!Double.isFinite(apogee)) {
            LOGGER.fine("No valid apogee estimate yet");
            return; // no estimate yet
        }

        if (apogee > targetApogeeMeters && !airbrakesExtended) {
            LOGGER.info("Extending airbrakes: predicted apogee " + apogee + 
                       " exceeds target " + targetApogeeMeters);
            if (context != null) {
                context.extend_airbrakes();
                LOGGER.fine("Airbrakes extension command sent");
            } else {
                LOGGER.warning("Context is null, cannot extend airbrakes");
            }
            airbrakesExtended = true;
        } else if (apogee <= targetApogeeMeters && airbrakesExtended) {
            LOGGER.info("Retracting airbrakes: predicted apogee " + apogee + 
                       " below or equal to target " + targetApogeeMeters);
            if (context != null) {
                context.retract_airbrakes();
                context.switch_altitude_back_to_pressure();
                LOGGER.fine("Airbrakes retraction and altitude sensor switch commands sent");
            } else {
                LOGGER.warning("Context is null, cannot retract airbrakes");
            }
            airbrakesExtended = false;
        }
    }

    public boolean isAirbrakesExtended() { 
        LOGGER.finest("Airbrakes extended status requested: " + airbrakesExtended);
        return airbrakesExtended; 
    }
    
    public double getTargetApogeeMeters() { 
        LOGGER.finest("Target apogee requested: " + targetApogeeMeters);
        return targetApogeeMeters; 
    }
}
