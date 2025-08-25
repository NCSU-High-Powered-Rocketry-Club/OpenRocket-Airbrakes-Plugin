package com.airbrakesplugin;

import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import info.openrocket.core.simulation.listeners.SimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Simulation‑level wrapper that injects {@link AirbrakeSimulationListener}
 * into each run and exposes bean properties for the GUI.
 */
public class AirbrakeExtension extends AbstractSimulationExtension {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeExtension.class);
    private final AirbrakeConfig config = new AirbrakeConfig();

    // =====================================================================
    @Override
    public String getName() {
        return "6DOF Airbrake Simulation";
    }

    @Override
    public String getDescription() {
        return "Adds 6DOF-aware airbrake simulation capabilities using CFD data.";
    }

    // =====================================================================
    @Override
    public void initialize(SimulationConditions conditions) throws SimulationException {
        log.info("Initializing AirbrakeExtension with config: {}", config);
        
        // Basic validation
        if (config.getCfdDataFilePath() == null || config.getCfdDataFilePath().trim().isEmpty()) {
            throw new SimulationException("CFD data file path is not configured for AirbrakeExtension.");
        }
        
        conditions.getSimulationListenerList().add((SimulationListener) new AirbrakeSimulationListener(config, conditions.getRocket()));
    }

    // =====================================================================
    public SimulationListener getSimulationListener(SimulationConditions conditions) {
        log.info("AirbrakeExtension: Creating simulation listener");
        
        Rocket rocket = (conditions != null) ? conditions.getRocket() : null;
        if (rocket == null) {
            log.warn("AirbrakeExtension: Rocket is null. Listener will be created with null rocket.");
        }
        
        return new AirbrakeSimulationListener(config, rocket);
    }

    // =====================================================================
    // Bean‑style properties (OpenRocket GUI binds via reflection)
    // =====================================================================

    public String getCfdDataFilePath() { 
        return config.getCfdDataFilePath(); 
    }
    public void setCfdDataFilePath(String p) {
        config.setCfdDataFilePath(p); fireChangeEvent(); 
    } 

    public double getReferenceArea() {
        return config.getReferenceArea(); 
    }
    public void setReferenceArea(double a) {
        config.setReferenceArea(a); fireChangeEvent(); 
    } 

    public double getReferenceLength() {
        return config.getReferenceLength(); 
    }
    public void setReferenceLength(double l) {
        config.setReferenceLength(l); fireChangeEvent(); 
    } 

    public double getTargetApogee() {
        return config.getTargetApogee(); 
    }
    public void setTargetApogee(double a) {
        config.setTargetApogee(a); fireChangeEvent(); 
    } 

    public double getDeployAltitudeThreshold() {
        return config.getDeployAltitudeThreshold(); 
    }
    public void setDeployAltitudeThreshold(double h) {
        config.setDeployAltitudeThreshold(h); fireChangeEvent(); 
    } 

    public double getMaxMachForDeployment() {
        return config.getMaxMachForDeployment(); 
    }
    public void setMaxMachForDeployment(double m) {
        config.setMaxMachForDeployment(m); fireChangeEvent(); 
    } 

    // ── Bang‑bang controller extras  ────────────────
    public boolean isAlwaysOpenMode() {
        return config.isAlwaysOpenMode(); 
    }
    public void setAlwaysOpenMode(boolean b) {
        config.setAlwaysOpenMode(b); fireChangeEvent(); 
    } 

    public double getAlwaysOpenPercentage() {
        return config.getAlwaysOpenPercentage(); 
    }
    public void setAlwaysOpenPercentage(double p) {
        config.setAlwaysOpenPercentage(p); fireChangeEvent(); 
    } 

    public double getApogeeToleranceMeters() {
        return config.getApogeeToleranceMeters();
    }
    public void setApogeeToleranceMeters(double tol) {
        config.setApogeeToleranceMeters(tol); fireChangeEvent(); 
    }

    @Override
    public String toString() { 
        return config.toString(); 
    }
}
