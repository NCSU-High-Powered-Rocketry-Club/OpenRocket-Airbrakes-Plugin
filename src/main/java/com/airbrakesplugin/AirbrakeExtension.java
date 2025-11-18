package com.airbrakesplugin;

import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.ApogeePredictor;

/**
 * Simulation-level wrapper that injects AirbrakeSimulationListener into each run.
 */
public class AirbrakeExtension extends AbstractSimulationExtension {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeExtension.class);

    /** Backing bean for all physical configuration parameters. */
    private final AirbrakeConfig config = new AirbrakeConfig();

    // Debug / developer controls (used by configurator & listener/controller)
    private boolean debugEnabled        = false;
    private boolean dbgAlwaysOpen       = false;
    private double  dbgForcedDeployFrac = 1.0;
    private boolean dbgTracePredictor   = true;
    private boolean dbgTraceController  = true;
    private boolean dbgWriteCsv         = true;
    private String  dbgCsvDir           = "";
    private boolean dbgShowConsole      = false;

    public AirbrakeExtension() {
        super();
    }

    /**
     * OpenRocket 23.09+ entry point:
     * called once when a simulation run starts.
     * Attach our listener here.
     */
    @Override
    public void initialize(final SimulationConditions conditions) throws SimulationException {
        try {
            // Aerobrake aerodynamics from CFD file
            final AirbrakeAerodynamics airbrakes =
                    new AirbrakeAerodynamics(config.getCfdDataFilePath());

            // RK4 apogee predictor (pure vertical model)
            final ApogeePredictor predictor = new ApogeePredictor();

            // Minimal bang-bang controller (context will be set by the listener on start)
            final AirbrakeController.ControlContext noopCtx =
                    new AirbrakeController.ControlContext() {
                        @Override public void extend_airbrakes() { }
                        @Override public void retract_airbrakes() { }
                        @Override public void switch_altitude_back_to_pressure() { }
                    };

            final double targetApogee = config.getTargetApogee();
            final AirbrakeController controller =
                    new AirbrakeController(targetApogee, predictor, noopCtx);

            // Reference area fallback for ΔCD conversion
            final double refAreaFallback = Math.max(1e-9, config.getReferenceArea());

            // Create listener and attach to this simulation
            final AirbrakeSimulationListener listener =
                    new AirbrakeSimulationListener(airbrakes, controller, predictor,
                                                   refAreaFallback, config);

            conditions.getSimulationListenerList().add(listener);
            log.info("Airbrakes extension initialized, listener attached");
        } catch (Exception e) {
            // Wrap any lower-level failure as a SimulationException
            throw new SimulationException("Failed to initialize Airbrakes plugin", e);
        }
    }

    // -------------------------------------------------------------------------
    // Optional metadata
    // -------------------------------------------------------------------------

    @Override
    public String getName() {
        // This is mostly cosmetic; the provider already gives a nice menu name.
        return "Airbrakes Plugin";
    }

    @Override
    public String getDescription() {
        return "Deployable airbrakes with RK4 apogee prediction and bang-bang control.";
    }

    // -------------------------------------------------------------------------
    // Bean-style properties for GUI (used via reflection in AirbrakeConfigurator)
    // Each setter calls fireChangeEvent() so OR knows the simulation config changed.
    // -------------------------------------------------------------------------

    public String getCfdDataFilePath() {
        return config.getCfdDataFilePath();
    }

    public void setCfdDataFilePath(String p) {
        config.setCfdDataFilePath(p);
        fireChangeEvent();
    }

    public double getReferenceArea() {
        return config.getReferenceArea();
    }

    public void setReferenceArea(double a) {
        config.setReferenceArea(a);
        fireChangeEvent();
    }

    public double getReferenceLength() {
        return config.getReferenceLength();
    }

    public void setReferenceLength(double l) {
        config.setReferenceLength(l);
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

    public void setTargetApogee(double a) {
        config.setTargetApogee(a);
        fireChangeEvent();
    }


    public double getMaxMachForDeployment() {
        return config.getMaxMachForDeployment();
    }

    public void setMaxMachForDeployment(double m) {
        config.setMaxMachForDeployment(m);
        fireChangeEvent();
    }

    public boolean isAlwaysOpenMode() {
        return config.isAlwaysOpenMode();
    }

    public void setAlwaysOpenMode(boolean v) {
        config.setAlwaysOpenMode(v);
        fireChangeEvent();
    }

    public double getAlwaysOpenPercentage() {
        return config.getAlwaysOpenPercentage();
    }

    public void setAlwaysOpenPercentage(double v) {
        config.setAlwaysOpenPercentage(v);
        fireChangeEvent();
    }

    public double getApogeeToleranceMeters() {
        return config.getApogeeToleranceMeters();
    }

    public void setApogeeToleranceMeters(double v) {
        config.setApogeeToleranceMeters(v);
        fireChangeEvent();
    }

    public boolean isDeployAfterBurnoutOnly() {
        return config.isDeployAfterBurnoutOnly();
    }

    public void setDeployAfterBurnoutOnly(boolean v) {
        config.setDeployAfterBurnoutOnly(v);
        fireChangeEvent();
    }

    public double getDeployAfterBurnoutDelayS() {
        return config.getDeployAfterBurnoutDelayS();
    }

    public void setDeployAfterBurnoutDelayS(double v) {
        config.setDeployAfterBurnoutDelayS(v);
        fireChangeEvent();
    }

    // -------------------------------------------------------------------------
    // Debug controls (wired from configurator into controller/listener as needed)
    // -------------------------------------------------------------------------

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

    public String getDbgCsvDir() {
        return (dbgCsvDir == null ? "" : dbgCsvDir);
    }

    public void setDbgCsvDir(String v) {
        this.dbgCsvDir = (v == null ? "" : v);
    }

    public boolean isDbgShowConsole() { return dbgShowConsole; }
    public void setDbgShowConsole(boolean v) { this.dbgShowConsole = v; }

    @Override
    public String toString() {
        return config.toString();
    }
}
