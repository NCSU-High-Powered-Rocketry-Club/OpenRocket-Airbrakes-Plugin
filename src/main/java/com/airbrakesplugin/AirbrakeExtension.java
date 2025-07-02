package com.airbrakesplugin;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.SimulationListener;
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
    public void initialize(SimulationConditions conditions) throws SimulationException {
        log.info("Initializing AirbrakeExtension with config: {}", config);
        conditions.getSimulationListenerList() .add((SimulationListener) new AirbrakeSimulationListener(config, conditions.getRocket()));
    }

    // =====================================================================
    // Bean‑style properties (OpenRocket GUI binds via reflection)
    // =====================================================================

    public String getCfdDataFilePath() { 
        return config.getCfdDataFilePath(); 
    }
    public void   setCfdDataFilePath(String p) {
        config.setCfdDataFilePath(p); fireChangeEvent(); 
    } 

   public double getReferenceArea() {
        return config.getReferenceArea(); 
    }
    public void   setReferenceArea(double a) {
        config.setReferenceArea(a); fireChangeEvent(); 
    } 

   public double getReferenceLength() {
        return config.getReferenceLength(); 
    }
    public void   setReferenceLength(double l) {
        config.setReferenceLength(l); fireChangeEvent(); 
    } 

   public double getMaxDeploymentRate() {
        return config.getMaxDeploymentRate(); 
    }
    public void   setMaxDeploymentRate(double r) {
        config.setMaxDeploymentRate(r); fireChangeEvent(); 
    } 

   public double getTargetApogee() {
        return config.getTargetApogee(); 
    }
    public void   setTargetApogee(double a) {
        config.setTargetApogee(a); fireChangeEvent(); 
    } 

   public double getDeployAltitudeThreshold() {
        return config.getDeployAltitudeThreshold(); 
    }
    public void   setDeployAltitudeThreshold(double h) {
        config.setDeployAltitudeThreshold(h); fireChangeEvent(); 
    } 

   public double getMaxMachForDeployment() {
        return config.getMaxMachForDeployment(); 
    }
    public void   setMaxMachForDeployment(double m) {
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
        return config.getApogeeToleranceMeters().orElse(null); 
    }
    public void setApogeeToleranceMeters(Double tol) {
        config.setApogeeToleranceMeters(tol); fireChangeEvent(); 
    }

    @Override
    public String toString() { return config.toString(); 
    }
}
