package com.airbrakesplugin; 

import net.sf.openrocket.simulation.SimulationStatus;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Simple airbrake controller logic.
 * Determines the desired airbrake deployment level.
 */
public class AirbrakeController {
    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);
    private final AirbrakeConfig config;

    public AirbrakeController(AirbrakeConfig config) {
        if (config == null) {
            log.error("AirbrakeController initialized with null config. Using default config.");
            this.config = new AirbrakeConfig(); 
        } else {
            this.config = config;
        }
    }

    public double getCommandedDeployment(double currentAltitude, double verticalVelocity, double currentMach,
                                         double currentDeployment, SimulationStatus status) {

        if (status == null) { 
            log.warn("AirbrakeController.getCommandedDeployment called with null SimulationStatus.");
            return 0.0; 
        }
         if (config == null) { 
            log.warn("AirbrakeController.getCommandedDeployment called with null config.");
            return 0.0; 
        }

        if (config.getMaxMachForDeployment() > 0 && currentMach > config.getMaxMachForDeployment()) {
            return 0.0; 
        }

        if (verticalVelocity <= 0 || currentAltitude < config.getDeployAltitudeThreshold()) {
             if (verticalVelocity <=0 && currentDeployment > 0.001) { 
                log.debug("Descending (VelZ: {} m/s) or below threshold altitude ({}m < {}m). Commanding 0% deployment.", 
                          verticalVelocity, currentAltitude, config.getDeployAltitudeThreshold());
             }
            return 0.0;
        }
        
        if (currentAltitude > config.getDeployAltitudeThreshold() && verticalVelocity > 10 && 
            (config.getMaxMachForDeployment() <= 0 || currentMach < config.getMaxMachForDeployment())) {
            if (config.getTargetApogee() > 0 && currentAltitude > (config.getTargetApogee() * 0.66) && verticalVelocity > 50) {
                return 0.75; 
            } else if (verticalVelocity > 20) { 
                return 0.25; 
            }
        }
        return 0.0; 
    }
}