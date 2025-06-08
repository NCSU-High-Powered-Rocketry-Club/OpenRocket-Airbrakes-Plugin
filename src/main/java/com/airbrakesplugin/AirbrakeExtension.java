package com.airbrakesplugin;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.SimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Simulation extension entry point; registers the 6DOF airbrake listener.
 */
public class AirbrakeExtension extends AbstractSimulationExtension {
    private static final Logger log = LoggerFactory.getLogger(AirbrakeExtension.class);
    private AirbrakeConfig config;

    public AirbrakeExtension() {
        this.config = new AirbrakeConfig();
        log.info("AirbrakeExtension created with config: {}", config);
    }

    @Override
    public void initialize(SimulationConditions conditions) throws SimulationException {
        log.info("Initializing AirbrakeExtension with config: {}", config);
        // Add our listener into the simulation's listener list
        conditions.getSimulationListenerList()
                  .add((SimulationListener) new AirbrakeSimulationListener(config, conditions.getRocket()));
    }

    // --- Bean properties for GUI binding ---

    public String getCfdDataFilePath() {
        return config.getCfdDataFilePath();
    }
    public void setCfdDataFilePath(String path) {
        config.setCfdDataFilePath(path);
        fireChangeEvent();
    }

    public double getReferenceArea() {
        return config.getReferenceArea();
    }
    public void setReferenceArea(double area) {
        config.setReferenceArea(area);
        fireChangeEvent();
    }

    public double getReferenceLength() {
        return config.getReferenceLength();
    }
    public void setReferenceLength(double length) {
        config.setReferenceLength(length);
        fireChangeEvent();
    }

    public double getMaxDeploymentRate() {
        return config.getMaxDeploymentRate();
    }
    public void setMaxDeploymentRate(double rate) {
        config.setMaxDeploymentRate(rate);
        fireChangeEvent();
    }

    public double getTargetApogee() {
        return config.getTargetApogee();
    }
    public void setTargetApogee(double apogee) {
        config.setTargetApogee(apogee);
        fireChangeEvent();
    }

    public double getDeployAltitudeThreshold() {
        return config.getDeployAltitudeThreshold();
    }
    public void setDeployAltitudeThreshold(double threshold) {
        config.setDeployAltitudeThreshold(threshold);
        fireChangeEvent();
    }

    public double getMaxMachForDeployment() {
        return config.getMaxMachForDeployment();
    }
    public void setMaxMachForDeployment(double mach) {
        config.setMaxMachForDeployment(mach);
        fireChangeEvent();
    }

    @Override
    public String toString() {
        return config.toString();
    }
}
