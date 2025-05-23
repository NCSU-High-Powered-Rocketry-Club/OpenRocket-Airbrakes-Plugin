package com.airbrakesplugin;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.SimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Represents an instance of the air-brake simulation extension.
 */
public class AirbrakeExtension extends AbstractSimulationExtension {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeExtension.class);

    // Configuration for the air-brake plug-in.
    private AirbrakeConfig config;

    public AirbrakeExtension() {
        // Default configuration
        this.config = new AirbrakeConfig();
        String defaultCsvPath = "airbrakes_cfd_data.csv";
        this.config.setCfdDataFilePath(defaultCsvPath);
        log.info("AirbrakeExtension created. Default CFD data path: {}", defaultCsvPath);
    }

    /* ------------------------------------------------------------------ */
    /*  METADATA                                                          */
    /* ------------------------------------------------------------------ */
    @Override
    public String getName() {
        return "6DOF Airbrake Simulation";
    }

    @Override
    public String getDescription() {
        return "Adds 6-DOF-aware air-brake simulation capabilities using CFD data.";
    }

    /* ------------------------------------------------------------------ */
    /*  SIMULATION INITIALIZATION HOOK                                    */
    /* ------------------------------------------------------------------ */
    /**
     * Called once before each simulation run.  Here we add our listener
     * into the SimulationConditions’ listener list.
     */
    @Override
    public void initialize(SimulationConditions conditions) throws SimulationException {
        if (config == null) {
            log.warn("AirbrakeExtension.initialize called with null config — recreating default.");
            config = new AirbrakeConfig();
        }
        log.info("Registering AirbrakeSimulationListener with config: {}", config);
        // This is the ONLY place you add your listener:
        conditions.getSimulationListenerList()
                  .add((SimulationListener) new AirbrakeSimulationListener(config, conditions.getRocket()));
    }

    /* ------------------------------------------------------------------ */
    /*  CONFIGURATION GETTERS / SETTERS                                   */
    /* ------------------------------------------------------------------ */
    public void setConfig(AirbrakeConfig config) {
        this.config = config;
    }

    public String getCfdDataFilePath()        { return config.getCfdDataFilePath(); }
    public void   setCfdDataFilePath(String p){ config.setCfdDataFilePath(p); }

    public double getReferenceArea()          { return config.getReferenceArea(); }
    public void   setReferenceArea(double a)  { config.setReferenceArea(a); }

    public double getReferenceLength()        { return config.getReferenceLength(); }
    public void   setReferenceLength(double l){ config.setReferenceLength(l); }

    public double getMaxDeploymentRate()      { return config.getMaxDeploymentRate(); }
    public void   setMaxDeploymentRate(double r){ config.setMaxDeploymentRate(r); }
}
