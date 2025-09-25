package com.airbrakesplugin;

import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import info.openrocket.core.simulation.listeners.SimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Simulation-level wrapper that injects AirbrakeSimulationListener into each run.
 */
public class AirbrakeExtension extends AbstractSimulationExtension {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeExtension.class);
    private final AirbrakeConfig config = new AirbrakeConfig();

    @Override public String getName() { return "6DOF Airbrake Simulation"; }
    @Override public String getDescription() { return "Adds 6DOF-aware airbrake simulation using CFD ΔCd."; }

    @Override
    public void initialize(final SimulationConditions conditions) throws SimulationException {
        if (config.getCfdDataFilePath() == null || config.getCfdDataFilePath().trim().isEmpty())
            throw new SimulationException("CFD data file path is not configured for AirbrakeExtension.");

        final SimulationListener listener = buildListener();
        conditions.getSimulationListenerList().add(listener);
    }

    public SimulationListener getSimulationListener(final SimulationConditions conditions) {
        return buildListener();
    }

    private SimulationListener buildListener() {
        try {
            // Aerobrake aerodynamics from CFD file)
            final AirbrakeAerodynamics airbrakes = new AirbrakeAerodynamics(config.getCfdDataFilePath());

            // Predictor
            final com.airbrakesplugin.util.ApogeePredictor predictor = new com.airbrakesplugin.util.ApogeePredictor();

            // Minimal bang-bang controller (context will be set by the listener on start)
            final AirbrakeController.ControlContext noopCtx = new AirbrakeController.ControlContext() {
                @Override public void extend_airbrakes() {}
                @Override public void retract_airbrakes() {}
                @Override public void switch_altitude_back_to_pressure() {}
            };
            
            final AirbrakeController controller =
                    new AirbrakeController(config.getTargetApogee(), predictor, noopCtx);

            // Gating Parameters 
            final double refAreaFallback = Math.max(1e-9, config.getReferenceArea());

            // Listener
            return new AirbrakeSimulationListener( airbrakes, controller, predictor, refAreaFallback, config);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create AirbrakeSimulationListener: " + e.getMessage(), e);
        }
    }

    // Bean-style properties for GUI

    public String getCfdDataFilePath() { return config.getCfdDataFilePath(); }
    public void setCfdDataFilePath(String p) { config.setCfdDataFilePath(p); fireChangeEvent(); }

    public double getReferenceArea() { return config.getReferenceArea(); }
    public void setReferenceArea(double a) { config.setReferenceArea(a); fireChangeEvent(); }

    public double getReferenceLength() { return config.getReferenceLength(); }
    public void setReferenceLength(double l) { config.setReferenceLength(l); fireChangeEvent(); }

    public double getTargetApogee() { return config.getTargetApogee(); }
    public void setTargetApogee(double a) { config.setTargetApogee(a); fireChangeEvent(); }

    public double getDeployAltitudeThreshold() { return config.getDeployAltitudeThreshold(); }
    public void setDeployAltitudeThreshold(double h) { config.setDeployAltitudeThreshold(h); fireChangeEvent(); }

    public double getMaxMachForDeployment() { return config.getMaxMachForDeployment(); }
    public void setMaxMachForDeployment(double m) { config.setMaxMachForDeployment(m); fireChangeEvent(); }

    public boolean isAlwaysOpenMode() { return config.isAlwaysOpenMode(); }
    public void setAlwaysOpenMode(boolean b) { config.setAlwaysOpenMode(b); fireChangeEvent(); }

    public double getAlwaysOpenPercentage() { return config.getAlwaysOpenPercentage(); }
    public void setAlwaysOpenPercentage(double p) { config.setAlwaysOpenPercentage(p); fireChangeEvent(); }

    public double getApogeeToleranceMeters() { return config.getApogeeToleranceMeters(); }
    public void setApogeeToleranceMeters(double tol) { config.setApogeeToleranceMeters(tol); fireChangeEvent(); }

    // Debug options
    private boolean debugEnabled = false;
    private boolean dbgAlwaysOpen = false;
    private double  dbgForcedDeployFrac = 1.0;
    private boolean dbgTracePredictor = true;
    private boolean dbgTraceController = true;
    private boolean dbgWriteCsv = true;
    private String  dbgCsvDir = "";
    private boolean dbgShowConsole = false;

    public boolean isDebugEnabled() { return debugEnabled; }
    public void setDebugEnabled(boolean v) { this.debugEnabled = v; }

    public boolean isDbgAlwaysOpen() { return dbgAlwaysOpen; }
    public void setDbgAlwaysOpen(boolean v) { this.dbgAlwaysOpen = v; }

    public double getDbgForcedDeployFrac() { return dbgForcedDeployFrac; }
    public void setDbgForcedDeployFrac(double v) {
        if (!Double.isFinite(v)) v = 0.0;
        this.dbgForcedDeployFrac = Math.max(0.0, Math.min(1.0, v));
    }

    public boolean isDbgTracePredictor() { return dbgTracePredictor; }
    public void setDbgTracePredictor(boolean v) { this.dbgTracePredictor = v; }

    public boolean isDbgTraceController() { return dbgTraceController; }
    public void setDbgTraceController(boolean v) { this.dbgTraceController = v; }

    public boolean isDbgWriteCsv() { return dbgWriteCsv; }
    public void setDbgWriteCsv(boolean v) { this.dbgWriteCsv = v; }

    public String getDbgCsvDir() { return (dbgCsvDir == null ? "" : dbgCsvDir); }
    public void setDbgCsvDir(String v) { this.dbgCsvDir = (v == null ? "" : v); }

    public boolean isDbgShowConsole() { return dbgShowConsole; }
    public void setDbgShowConsole(boolean v) { this.dbgShowConsole = v; }

    @Override public String toString() { return config.toString(); }
}
