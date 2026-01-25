package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Method;

/**
 * Simulation-level wrapper that injects {@link AirbrakeSimulationListener} into each run.
 *
 * <p><b>Persistence (run-to-run / .ork save-load):</b>
 * OpenRocket persists a simulation extension's settings into the <code>.ork</code> file via the
 * {@link AbstractSimulationExtension}-provided {@code config} object.
 *
 * <ul>
 *   <li>Read values with {@code config.getDouble(...)} / {@code config.getString(...)}.</li>
 *   <li>Write values with {@code config.put(...)}.</li>
 *   <li>After every UI-driven setter, call {@code fireChangeEvent()} so the Simulation is marked dirty
 *       and the updated config is serialized into the <code>.ork</code>.</li>
 * </ul>
 *
 * <p>
 * IMPORTANT: do <b>not</b> cache the config values at construction time. OpenRocket may populate the
 * extension's {@code config} after instantiation when loading a <code>.ork</code>; caching too early
 * can cause saved values to be ignored. Therefore, getters read directly from {@code config}.
 * </p>
 */
public class AirbrakeExtension extends AbstractSimulationExtension {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeExtension.class);

    // ---------------------------------------------------------------------
    // Config keys (persisted in .ork)
    //
    // NOTE: Keys are stored per-extension instance, but we still prefix for clarity.
    // The *_LEGACY keys are for backward compatibility with earlier builds.
    // ---------------------------------------------------------------------

    private static final String CFG_PREFIX = "airbrakes.";

    private static final String K_CFD_DATA_FILE_PATH       = CFG_PREFIX + "cfdDataFilePath";
    private static final String K_REFERENCE_AREA           = CFG_PREFIX + "referenceArea";
    private static final String K_REFERENCE_LENGTH         = CFG_PREFIX + "referenceLength";
    private static final String K_MAX_DEPLOYMENT_RATE      = CFG_PREFIX + "maxDeploymentRate";
    private static final String K_TARGET_APOGEE            = CFG_PREFIX + "targetApogee";
    private static final String K_MAX_MACH_FOR_DEPLOYMENT  = CFG_PREFIX + "maxMachForDeployment";
    private static final String K_ALWAYS_OPEN_MODE         = CFG_PREFIX + "alwaysOpenMode";
    private static final String K_ALWAYS_OPEN_PERCENT      = CFG_PREFIX + "alwaysOpenPercentage";
    private static final String K_APOGEE_TOLERANCE_M        = CFG_PREFIX + "apogeeToleranceMeters";
    private static final String K_DEPLOY_AFTER_BURNOUT     = CFG_PREFIX + "deployAfterBurnoutOnly";
    private static final String K_DEPLOY_BURNOUT_DELAY_S   = CFG_PREFIX + "deployAfterBurnoutDelayS";

    // Debug keys
    private static final String K_DBG_ENABLED              = CFG_PREFIX + "debugEnabled";
    private static final String K_DBG_ALWAYS_OPEN          = CFG_PREFIX + "dbgAlwaysOpen";
    private static final String K_DBG_FORCED_DEPLOY_FRAC   = CFG_PREFIX + "dbgForcedDeployFrac";
    private static final String K_DBG_TRACE_PREDICTOR      = CFG_PREFIX + "dbgTracePredictor";
    private static final String K_DBG_TRACE_CONTROLLER     = CFG_PREFIX + "dbgTraceController";
    private static final String K_DBG_WRITE_CSV            = CFG_PREFIX + "dbgWriteCsv";
    private static final String K_DBG_CSV_DIR              = CFG_PREFIX + "dbgCsvDir";
    private static final String K_DBG_SHOW_CONSOLE         = CFG_PREFIX + "dbgShowConsole";

    // Legacy (un-prefixed) keys (older plugin builds)
    private static final String K_CFD_DATA_FILE_PATH_LEGACY      = "cfdDataFilePath";
    private static final String K_REFERENCE_AREA_LEGACY          = "referenceArea";
    private static final String K_REFERENCE_LENGTH_LEGACY        = "referenceLength";
    private static final String K_MAX_DEPLOYMENT_RATE_LEGACY     = "maxDeploymentRate";
    private static final String K_TARGET_APOGEE_LEGACY           = "targetApogee";
    private static final String K_MAX_MACH_FOR_DEPLOYMENT_LEGACY = "maxMachForDeployment";
    private static final String K_ALWAYS_OPEN_MODE_LEGACY        = "alwaysOpenMode";
    private static final String K_ALWAYS_OPEN_PERCENT_LEGACY     = "alwaysOpenPercentage";
    private static final String K_APOGEE_TOLERANCE_M_LEGACY       = "apogeeToleranceMeters";
    private static final String K_DEPLOY_AFTER_BURNOUT_LEGACY     = "deployAfterBurnoutOnly";
    private static final String K_DEPLOY_BURNOUT_DELAY_S_LEGACY   = "deployAfterBurnoutDelayS";
    private static final String K_DBG_ENABLED_LEGACY              = "debugEnabled";
    private static final String K_DBG_ALWAYS_OPEN_LEGACY          = "dbgAlwaysOpen";
    private static final String K_DBG_FORCED_DEPLOY_FRAC_LEGACY   = "dbgForcedDeployFrac";
    private static final String K_DBG_TRACE_PREDICTOR_LEGACY      = "dbgTracePredictor";
    private static final String K_DBG_TRACE_CONTROLLER_LEGACY     = "dbgTraceController";
    private static final String K_DBG_WRITE_CSV_LEGACY            = "dbgWriteCsv";
    private static final String K_DBG_CSV_DIR_LEGACY              = "dbgCsvDir";
    private static final String K_DBG_SHOW_CONSOLE_LEGACY         = "dbgShowConsole";

    // Defaults (must match AirbrakeConfig defaults)
    private static final String D_CFD_DATA_FILE_PATH = "";
    private static final double D_REFERENCE_AREA = 0.0;
    private static final double D_REFERENCE_LENGTH = 0.0;
    private static final double D_MAX_DEPLOYMENT_RATE = 40.0;
    private static final double D_TARGET_APOGEE = 0.0;
    private static final double D_MAX_MACH_FOR_DEPLOYMENT = 1.0;
    private static final boolean D_ALWAYS_OPEN_MODE = false;
    private static final double D_ALWAYS_OPEN_PERCENT = 1.0;
    private static final double D_APOGEE_TOLERANCE_M = 5.0;
    private static final boolean D_DEPLOY_AFTER_BURNOUT = false;
    private static final double D_DEPLOY_BURNOUT_DELAY_S = 0.0;

    private static final boolean D_DBG_ENABLED = false;
    private static final boolean D_DBG_ALWAYS_OPEN = false;
    private static final double D_DBG_FORCED_DEPLOY_FRAC = 1.0;
    private static final boolean D_DBG_TRACE_PREDICTOR = true;
    private static final boolean D_DBG_TRACE_CONTROLLER = true;
    private static final boolean D_DBG_WRITE_CSV = true;
    private static final String  D_DBG_CSV_DIR = "";
    private static final boolean D_DBG_SHOW_CONSOLE = false;

    // ---------------------------------------------------------------------
    // Config helpers (kept similar to MonteCarloExtension for OR version tolerance)
    // ---------------------------------------------------------------------

    private static double clamp01(double v) {
        if (!Double.isFinite(v)) return 0.0;
        return Math.max(0.0, Math.min(1.0, v));
    }

    private static String safeString(String s) {
        return (s == null) ? "" : s;
    }

    private Object invokeConfigGetRaw(String key) {
        if (key == null) return null;
        try {
            Method m = config.getClass().getMethod("get", String.class);
            return m.invoke(config, key);
        } catch (Exception ignored) {
            return null;
        }
    }

    private boolean invokeConfigPut(String key, Object value) {
        if (key == null) return false;
        try {
            Method m = config.getClass().getMethod("put", String.class, Object.class);
            m.invoke(config, key, value);
            return true;
        } catch (Exception ignored) {
            // fall through
        }
        // Try primitive overloads if present
        if (value instanceof Boolean b) {
            try {
                Method m = config.getClass().getMethod("put", String.class, boolean.class);
                m.invoke(config, key, b.booleanValue());
                return true;
            } catch (Exception ignored) { }
        }
        if (value instanceof Number n) {
            try {
                Method m = config.getClass().getMethod("put", String.class, double.class);
                m.invoke(config, key, n.doubleValue());
                return true;
            } catch (Exception ignored) { }
        }
        if (value instanceof String s) {
            try {
                Method m = config.getClass().getMethod("put", String.class, String.class);
                m.invoke(config, key, s);
                return true;
            } catch (Exception ignored) { }
        }
        return false;
    }

    private double cfgDouble(String key, String legacyKey, double fallback) {
        Object v = invokeConfigGetRaw(key);
        if (v == null && legacyKey != null) v = invokeConfigGetRaw(legacyKey);
        if (v instanceof Number n) return n.doubleValue();
        if (v instanceof String s) {
            try { return Double.parseDouble(s.trim()); } catch (Exception ignored) { }
        }
        // Fall back to getDouble if available
        try {
            return config.getDouble(key, fallback);
        } catch (Throwable ignored) {
            try {
                return (legacyKey != null) ? config.getDouble(legacyKey, fallback) : fallback;
            } catch (Throwable ignored2) {
                return fallback;
            }
        }
    }

    private String cfgString(String key, String legacyKey, String fallback) {
        Object v = invokeConfigGetRaw(key);
        if (v == null && legacyKey != null) v = invokeConfigGetRaw(legacyKey);
        if (v != null && !(v instanceof Number) && !(v instanceof Boolean)) return String.valueOf(v);

        // Try getString(key, fallback) if present
        try {
            Method m = config.getClass().getMethod("getString", String.class, String.class);
            Object out = m.invoke(config, key, fallback);
            if (out != null) return out.toString();
        } catch (Exception ignored) { }
        if (legacyKey != null) {
            try {
                Method m = config.getClass().getMethod("getString", String.class, String.class);
                Object out = m.invoke(config, legacyKey, fallback);
                if (out != null) return out.toString();
            } catch (Exception ignored) { }
        }

        return fallback;
    }

    private boolean cfgBool(String key, String legacyKey, boolean fallback) {
        Object v = invokeConfigGetRaw(key);
        if (v == null && legacyKey != null) v = invokeConfigGetRaw(legacyKey);
        if (v instanceof Boolean b) return b;
        if (v instanceof Number n) return n.doubleValue() != 0.0;
        if (v instanceof String s) {
            if ("true".equalsIgnoreCase(s) || "false".equalsIgnoreCase(s)) return Boolean.parseBoolean(s);
            try { return Double.parseDouble(s.trim()) != 0.0; } catch (Exception ignored) { }
        }
        return fallback;
    }

    private void cfgPutDouble(String key, double value) {
        if (invokeConfigPut(key, value)) return;
        try { config.put(key, value); } catch (Throwable ignored) { }
    }

    private void cfgPutString(String key, String value) {
        value = safeString(value);
        if (invokeConfigPut(key, value)) return;
        try { config.put(key, value); } catch (Throwable ignored) { }
    }

    private void cfgPutBool(String key, boolean value) {
        if (invokeConfigPut(key, value)) return;
        // As a last resort, store as string for max compatibility
        cfgPutString(key, value ? "true" : "false");
    }

    // ---------------------------------------------------------------------
    // OpenRocket lifecycle
    // ---------------------------------------------------------------------

    /**
     * Called once when a simulation run starts. Attaches our listener.
     */
    @Override
    public void initialize(final SimulationConditions conditions) throws SimulationException {
        try {
            // Build a runtime bean from the persisted config (source of truth)
            final AirbrakeConfig runtimeCfg = new AirbrakeConfig();
            runtimeCfg.setCfdDataFilePath(getCfdDataFilePath());
            runtimeCfg.setReferenceArea(getReferenceArea());
            runtimeCfg.setReferenceLength(getReferenceLength());
            runtimeCfg.setMaxDeploymentRate(getMaxDeploymentRate());
            runtimeCfg.setTargetApogee(getTargetApogee());
            runtimeCfg.setMaxMachForDeployment(getMaxMachForDeployment());
            runtimeCfg.setAlwaysOpenMode(isAlwaysOpenMode());
            runtimeCfg.setAlwaysOpenPercentage(getAlwaysOpenPercentage());
            runtimeCfg.setApogeeToleranceMeters(getApogeeToleranceMeters());
            runtimeCfg.setDeployAfterBurnoutOnly(isDeployAfterBurnoutOnly());
            runtimeCfg.setDeployAfterBurnoutDelayS(getDeployAfterBurnoutDelayS());

            runtimeCfg.setDebugEnabled(isDebugEnabled());
            runtimeCfg.setDbgAlwaysOpen(isDbgAlwaysOpen());
            runtimeCfg.setDbgForcedDeployFrac(getDbgForcedDeployFrac());
            runtimeCfg.setDbgTracePredictor(isDbgTracePredictor());
            runtimeCfg.setDbgTraceController(isDbgTraceController());
            runtimeCfg.setDbgWriteCsv(isDbgWriteCsv());
            runtimeCfg.setDbgCsvDir(getDbgCsvDir());
            runtimeCfg.setDbgShowConsole(isDbgShowConsole());

            // Aerobrake aerodynamics from CFD file
            final AirbrakeAerodynamics airbrakes = new AirbrakeAerodynamics(runtimeCfg.getCfdDataFilePath());

            // RK4 apogee predictor (pure vertical model)
            final ApogeePredictor predictor = new ApogeePredictor();

            // Minimal bang-bang controller (context will be set by the listener on start)
            final AirbrakeController.ControlContext noopCtx = new AirbrakeController.ControlContext() {
                @Override public void extend_airbrakes() { }
                @Override public void retract_airbrakes() { }
                @Override public void switch_altitude_back_to_pressure() { }
            };

            final AirbrakeController controller = new AirbrakeController(runtimeCfg.getTargetApogee(), predictor, noopCtx);

            // Reference area fallback for ΔCD conversion
            final double refAreaFallback = Math.max(1e-9, runtimeCfg.getReferenceArea());

            final AirbrakeSimulationListener listener = new AirbrakeSimulationListener(
                    airbrakes,
                    controller,
                    predictor,
                    refAreaFallback,
                    runtimeCfg
            );

            conditions.getSimulationListenerList().add(listener);
            log.info("Airbrakes extension initialized, listener attached");
        } catch (Exception e) {
            throw new SimulationException("Failed to initialize Airbrakes plugin", e);
        }
    }

    // ---------------------------------------------------------------------
    // Metadata
    // ---------------------------------------------------------------------

    @Override
    public String getName() {
        return "Airbrakes Plugin";
    }

    @Override
    public String getDescription() {
        return "Deployable airbrakes with RK4 apogee prediction and bang-bang control.";
    }

    // ---------------------------------------------------------------------
    // Bean-style properties for GUI (AirbrakeConfigurator uses reflection)
    // Every setter must: (1) write to persisted config, (2) call fireChangeEvent().
    // ---------------------------------------------------------------------

    public String getCfdDataFilePath() {
        return cfgString(K_CFD_DATA_FILE_PATH, K_CFD_DATA_FILE_PATH_LEGACY, D_CFD_DATA_FILE_PATH);
    }

    public void setCfdDataFilePath(String p) {
        cfgPutString(K_CFD_DATA_FILE_PATH, safeString(p));
        fireChangeEvent();
    }

    public double getReferenceArea() {
        return cfgDouble(K_REFERENCE_AREA, K_REFERENCE_AREA_LEGACY, D_REFERENCE_AREA);
    }

    public void setReferenceArea(double a) {
        cfgPutDouble(K_REFERENCE_AREA, a);
        fireChangeEvent();
    }

    public double getReferenceLength() {
        return cfgDouble(K_REFERENCE_LENGTH, K_REFERENCE_LENGTH_LEGACY, D_REFERENCE_LENGTH);
    }

    public void setReferenceLength(double l) {
        cfgPutDouble(K_REFERENCE_LENGTH, l);
        fireChangeEvent();
    }

    public double getMaxDeploymentRate() {
        return cfgDouble(K_MAX_DEPLOYMENT_RATE, K_MAX_DEPLOYMENT_RATE_LEGACY, D_MAX_DEPLOYMENT_RATE);
    }

    public void setMaxDeploymentRate(double rate) {
        cfgPutDouble(K_MAX_DEPLOYMENT_RATE, rate);
        fireChangeEvent();
    }

    public double getTargetApogee() {
        return cfgDouble(K_TARGET_APOGEE, K_TARGET_APOGEE_LEGACY, D_TARGET_APOGEE);
    }

    public void setTargetApogee(double a) {
        cfgPutDouble(K_TARGET_APOGEE, a);
        fireChangeEvent();
    }

    public double getMaxMachForDeployment() {
        return cfgDouble(K_MAX_MACH_FOR_DEPLOYMENT, K_MAX_MACH_FOR_DEPLOYMENT_LEGACY, D_MAX_MACH_FOR_DEPLOYMENT);
    }

    public void setMaxMachForDeployment(double m) {
        cfgPutDouble(K_MAX_MACH_FOR_DEPLOYMENT, m);
        fireChangeEvent();
    }

    public boolean isAlwaysOpenMode() {
        return cfgBool(K_ALWAYS_OPEN_MODE, K_ALWAYS_OPEN_MODE_LEGACY, D_ALWAYS_OPEN_MODE);
    }

    public void setAlwaysOpenMode(boolean v) {
        cfgPutBool(K_ALWAYS_OPEN_MODE, v);
        fireChangeEvent();
    }

    public double getAlwaysOpenPercentage() {
        return cfgDouble(K_ALWAYS_OPEN_PERCENT, K_ALWAYS_OPEN_PERCENT_LEGACY, D_ALWAYS_OPEN_PERCENT);
    }

    public void setAlwaysOpenPercentage(double v) {
        cfgPutDouble(K_ALWAYS_OPEN_PERCENT, clamp01(v));
        fireChangeEvent();
    }

    public double getApogeeToleranceMeters() {
        return cfgDouble(K_APOGEE_TOLERANCE_M, K_APOGEE_TOLERANCE_M_LEGACY, D_APOGEE_TOLERANCE_M);
    }

    public void setApogeeToleranceMeters(double v) {
        cfgPutDouble(K_APOGEE_TOLERANCE_M, v);
        fireChangeEvent();
    }

    public boolean isDeployAfterBurnoutOnly() {
        return cfgBool(K_DEPLOY_AFTER_BURNOUT, K_DEPLOY_AFTER_BURNOUT_LEGACY, D_DEPLOY_AFTER_BURNOUT);
    }

    public void setDeployAfterBurnoutOnly(boolean v) {
        cfgPutBool(K_DEPLOY_AFTER_BURNOUT, v);
        fireChangeEvent();
    }

    public double getDeployAfterBurnoutDelayS() {
        return cfgDouble(K_DEPLOY_BURNOUT_DELAY_S, K_DEPLOY_BURNOUT_DELAY_S_LEGACY, D_DEPLOY_BURNOUT_DELAY_S);
    }

    public void setDeployAfterBurnoutDelayS(double v) {
        cfgPutDouble(K_DEPLOY_BURNOUT_DELAY_S, Math.max(0.0, v));
        fireChangeEvent();
    }

    // ---------------------------------------------------------------------
    // Debug controls (persisted)
    // ---------------------------------------------------------------------

    public boolean isDebugEnabled() {
        return cfgBool(K_DBG_ENABLED, K_DBG_ENABLED_LEGACY, D_DBG_ENABLED);
    }

    public void setDebugEnabled(boolean v) {
        cfgPutBool(K_DBG_ENABLED, v);
        fireChangeEvent();
    }

    public boolean isDbgAlwaysOpen() {
        return cfgBool(K_DBG_ALWAYS_OPEN, K_DBG_ALWAYS_OPEN_LEGACY, D_DBG_ALWAYS_OPEN);
    }

    public void setDbgAlwaysOpen(boolean v) {
        cfgPutBool(K_DBG_ALWAYS_OPEN, v);
        fireChangeEvent();
    }

    public double getDbgForcedDeployFrac() {
        return clamp01(cfgDouble(K_DBG_FORCED_DEPLOY_FRAC, K_DBG_FORCED_DEPLOY_FRAC_LEGACY, D_DBG_FORCED_DEPLOY_FRAC));
    }

    public void setDbgForcedDeployFrac(double v) {
        cfgPutDouble(K_DBG_FORCED_DEPLOY_FRAC, clamp01(v));
        fireChangeEvent();
    }

    public boolean isDbgTracePredictor() {
        return cfgBool(K_DBG_TRACE_PREDICTOR, K_DBG_TRACE_PREDICTOR_LEGACY, D_DBG_TRACE_PREDICTOR);
    }

    public void setDbgTracePredictor(boolean v) {
        cfgPutBool(K_DBG_TRACE_PREDICTOR, v);
        fireChangeEvent();
    }

    public boolean isDbgTraceController() {
        return cfgBool(K_DBG_TRACE_CONTROLLER, K_DBG_TRACE_CONTROLLER_LEGACY, D_DBG_TRACE_CONTROLLER);
    }

    public void setDbgTraceController(boolean v) {
        cfgPutBool(K_DBG_TRACE_CONTROLLER, v);
        fireChangeEvent();
    }

    public boolean isDbgWriteCsv() {
        return cfgBool(K_DBG_WRITE_CSV, K_DBG_WRITE_CSV_LEGACY, D_DBG_WRITE_CSV);
    }

    public void setDbgWriteCsv(boolean v) {
        cfgPutBool(K_DBG_WRITE_CSV, v);
        fireChangeEvent();
    }

    public String getDbgCsvDir() {
        return cfgString(K_DBG_CSV_DIR, K_DBG_CSV_DIR_LEGACY, D_DBG_CSV_DIR);
    }

    public void setDbgCsvDir(String v) {
        cfgPutString(K_DBG_CSV_DIR, safeString(v));
        fireChangeEvent();
    }

    public boolean isDbgShowConsole() {
        return cfgBool(K_DBG_SHOW_CONSOLE, K_DBG_SHOW_CONSOLE_LEGACY, D_DBG_SHOW_CONSOLE);
    }

    public void setDbgShowConsole(boolean v) {
        cfgPutBool(K_DBG_SHOW_CONSOLE, v);
        fireChangeEvent();
    }

    @Override
    public String toString() {
        return "AirbrakeExtension{" +
                "cfdDataFilePath='" + getCfdDataFilePath() + '\'' +
                ", referenceArea=" + getReferenceArea() +
                ", referenceLength=" + getReferenceLength() +
                ", maxDeploymentRate=" + getMaxDeploymentRate() +
                ", targetApogee=" + getTargetApogee() +
                ", maxMachForDeployment=" + getMaxMachForDeployment() +
                ", alwaysOpenMode=" + isAlwaysOpenMode() +
                ", alwaysOpenPercentage=" + getAlwaysOpenPercentage() +
                ", apogeeToleranceMeters=" + getApogeeToleranceMeters() +
                ", deployAfterBurnoutOnly=" + isDeployAfterBurnoutOnly() +
                ", deployAfterBurnoutDelayS=" + getDeployAfterBurnoutDelayS() +
                ", debugEnabled=" + isDebugEnabled() +
                ", dbgAlwaysOpen=" + isDbgAlwaysOpen() +
                ", dbgForcedDeployFrac=" + getDbgForcedDeployFrac() +
                ", dbgTracePredictor=" + isDbgTracePredictor() +
                ", dbgTraceController=" + isDbgTraceController() +
                ", dbgWriteCsv=" + isDbgWriteCsv() +
                ", dbgCsvDir='" + safeString(getDbgCsvDir()) + '\'' +
                ", dbgShowConsole=" + isDbgShowConsole() +
                '}';
    }
}
