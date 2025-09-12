package com.airbrakesplugin;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.ApogeePredictor;
import com.airbrakesplugin.util.DebugTrace;

import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.util.Coordinate;

import java.lang.reflect.Method;
import java.util.Objects;

/**
 * AirbrakeSimulationListener — coefficient injection of ΔCd (and ΔCm)
 * plus robust actuator/command integration.
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // ---- config & sim objects ------------------------------------------------
    private final AirbrakeConfig config;
    private Rocket rocket;                // optional; not required for coeff updates
    private SimulationStatus sim;

    // ---- subsystems ----------------------------------------------------------
    private AirbrakeAerodynamics aerodynamics;     // ΔCd(M, deploy), optional ΔCm
    private ApogeePredictor      predictor;        // predictor (integrates accel incl. g)
    private AirbrakeController   controller;       // bang-bang controller (flexible gate)

    // ---- geometry & limits (SI) ---------------------------------------------
    private double Sref = 0.0;             // m²
    private double Lref = 0.0;             // m
    private double maxRate = 4.0;          // fraction/s (actuator slew)

    // ---- state ---------------------------------------------------------------
    private double lastTime_s = 0.0;
    private double deploy     = 0.0;       // physical actuator state [0..1]
    private double commandedDeploy = 0.0;  // controller desired end-state (0 or 1)
    private double currentAirbrakeDeployment = 0.0;

    // Tracking for accel computation & coast gating
    private Double lastVaxis = null;
    private boolean coastLatched = false;
    private int thrustZeroHits = 0;
    private int accelGateHits  = 0;

    // ---- coast detection thresholds -----------------------------------------
    private static final double G = 9.80665;
    private static final double THRUST_EPSILON_N     = 1.0;
    private static final int    THRUST_CONSEC_HITS   = 2;
    private static final double ACCEL_MINUS_G_THRESH = 1.5; // m/s^2
    private static final int    ACCEL_CONSEC_HITS    = 2;
    private static final double MIN_ASCEND_VZ        = 0.5; // m/s

    // ---- Debug / instrumentation --------------------------------------------
    private DebugTrace dbg = null;
    private boolean firstDeployMoveLogged = false;

    public AirbrakeSimulationListener(final AirbrakeConfig cfg, final Rocket rkt) {
        this.config = Objects.requireNonNull(cfg, "AirbrakeConfig must not be null");
        this.rocket = rkt;
        log.debug("[Airbrakes] Listener created with config: {}", cfg);
    }

    public double getCurrentAirbrakeDeployment() { return currentAirbrakeDeployment; }
    public boolean isAirbrakeCurrentlyDeployed() { return currentAirbrakeDeployment > 1e-6; }

    // ---- lifecycle: begin ----------------------------------------------------
    @Override
    public void startSimulation(final SimulationStatus status) throws SimulationException {
        this.sim = status;
        log.debug("[Airbrakes] Simulation starting...");
        try {
            this.rocket = (Rocket) status.getClass().getMethod("getRocket").invoke(status);
        } catch (Throwable ignore) {}

        setupDebug(config, tryGetSimName(status));

        try {
            this.aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception e) {
            throw new SimulationException("Airbrake CFD data failed to load", e);
        }

        this.predictor = new com.airbrakesplugin.util.ApogeePredictor();
        try {
            if (config != null && config.isDebugEnabled() && config.isDbgTracePredictor()) {
                if (dbg != null) {
                    predictor.setTraceSink((tt, alt, vz, az, apStrict, apBest, unc, packets, note) -> {
                        try { dbg.addPredictor(new DebugTrace.PredictorSample(tt, alt, vz, az, apStrict, apBest, unc, packets, note)); }
                        catch (Throwable ignore) {}
                    });
                } else {
                    predictor.setTraceSink(null);
                }
            } else {
                predictor.setTraceSink(null);
            }
        } catch (Throwable t) { log.debug("[Airbrakes] Predictor trace wiring skipped: {}", t.toString()); }

        final double target = safeTargetApogee(config);
        this.commandedDeploy = 0.0;
        this.controller = new AirbrakeController(target, predictor, new AirbrakeController.ControlContext() {
            @Override public void extend_airbrakes() { commandedDeploy = 1.0; }
            @Override public void retract_airbrakes() { commandedDeploy = 0.0; }
            @Override public void switch_altitude_back_to_pressure() { }
        }, .5, 5.0);

        try {
            Method m = config.getClass().getMethod("getApogeeToleranceMeters");
            Object v = m.invoke(config);
            if (v instanceof Number) controller.setApogeeDeadbandMeters(Math.max(0.0, ((Number) v).doubleValue()));
        } catch (Throwable ignore) {}

        Sref    = config.getReferenceArea();
        Lref    = config.getReferenceLength();
        maxRate = config.getMaxDeploymentRate();

        deploy        = 0.0;
        lastTime_s    = finiteOr(status.getSimulationTime(), 0.0);
        lastVaxis     = null;
        coastLatched  = false;
        thrustZeroHits = 0;
        accelGateHits  = 0;
        firstDeployMoveLogged = false;
    }

    // ---- lifecycle: step state update (predictor, controller, actuator) -----
    @Override
    public boolean preStep(final SimulationStatus status) {
        if (aerodynamics == null || controller == null || predictor == null) return true;

        final double t  = finiteOr(status.getSimulationTime(), lastTime_s);
        final double dt = Math.max(0.0, t - lastTime_s);
        lastTime_s = t;
        if (dt <= 0.0) return true;

        final Coordinate vWorld = status.getRocketVelocity();
        final double vAxis      = (vWorld != null) ? vWorld.z : 0.0;
        final double alt        = safeGet(status, FlightDataType.TYPE_ALTITUDE, 0.0);

        double aNetAxis = Double.NaN;
        if (lastVaxis != null) aNetAxis = (vAxis - lastVaxis) / Math.max(1e-6, dt);
        lastVaxis = vAxis;

        final double gAxis   = -G;
        final double aMinusG = Double.isFinite(aNetAxis) ? (aNetAxis - gAxis) : Double.NaN;

        boolean thrustGate = false, accelGate = false;
        final double thrust = safeGet(status, FlightDataType.TYPE_THRUST_FORCE, Double.NaN);
        if (Double.isFinite(thrust) && thrust <= THRUST_EPSILON_N) {
            thrustZeroHits++; thrustGate = (thrustZeroHits >= THRUST_CONSEC_HITS);
        } else thrustZeroHits = 0;

        if (Double.isFinite(aMinusG) && aMinusG <= ACCEL_MINUS_G_THRESH) {
            accelGateHits++; accelGate = (accelGateHits >= ACCEL_CONSEC_HITS);
        } else accelGateHits = 0;

        if (!coastLatched && (vAxis > MIN_ASCEND_VZ) && (thrustGate || accelGate)) {
            coastLatched = true;
            controller.notifyCoastLatched(status);
        }

        if (coastLatched && Double.isFinite(aNetAxis)) {
            predictor.update(aNetAxis, dt, alt, vAxis);
        }

        try { controller.updateAndGateFlexible(status); } catch (Throwable ignore) {}

        double beforeCmd = commandedDeploy;
        if (hasMethod(controller, "hasSetpoint") && hasMethod(controller, "currentSetpoint")) {
            try {
                if ((boolean) controller.getClass().getMethod("hasSetpoint").invoke(controller)) {
                    double u = (double) controller.getClass().getMethod("currentSetpoint").invoke(controller);
                    commandedDeploy = clamp01(u);
                }
            } catch (Throwable ignore) {}
        }

        double effectiveSetpoint = commandedDeploy;
        final double maxMach = Math.max(0.0, config.getMaxMachForDeployment());
        final double machNow = safeGet(status, FlightDataType.TYPE_MACH_NUMBER, Double.NaN);
        if (Double.isFinite(machNow) && machNow > maxMach && maxMach > 0.0) effectiveSetpoint = 0.0;

        final double dmax = maxRate * dt;
        final double delta = Math.max(-dmax, Math.min(dmax, effectiveSetpoint - deploy));
        final double oldDeploy = deploy;
        deploy = clamp01(deploy + delta);

        currentAirbrakeDeployment = deploy;
        tryEmitExtensionToFlightData(status, deploy);
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Waterloo-style override: set CDAxial from our airbrake drag (simple & fast)
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status,
                                                        final AerodynamicForces forces) {
        final double velocityZ = status.getRocketVelocity().z;

        // Override CD only during coast and at valid speeds
        if (isExtensionAllowed(status)) {
            // airbrake extension (use listener state)
            final double airbrakeExt = currentAirbrakeDeployment;

            // altitude (fallback to world-Z if site altitude unavailable)
            double altitude;
            try {
                altitude = status.getRocketPosition().z
                        + status.getSimulationConditions().getLaunchSite().getAltitude();
            } catch (Throwable t) {
                altitude = status.getRocketPosition().z;
            }

            // ΔCd from LUT at current Mach/extension
            final double v2 = status.getRocketVelocity().length2();
            final double v  = Math.sqrt(Math.max(0.0, v2));
            final double M  = AirDensity.machFromV(v, altitude);
            final double dCd = aerodynamics.getIncrementalCd(M, airbrakeExt);

            // q (compressibility-aware) and reference area
            final double rhoEff = AirDensity.rhoForDynamicPressureFromV(altitude, v);
            final double dynP   = 0.5 * rhoEff * v2;
            final double refA   = Math.max(1e-9, Sref);

            // Equivalent force then back to CDAxial; this equals dCd but mirrors the pattern
            final double dragForce = dCd * dynP * refA;
            final double cDAxial   = dragForce / dynP / refA;

            // Set CDAxial (fall back across possible setter names)
            trySetDouble(forces, cDAxial,
                    "setCDAxial", "setAxialForceCoefficient", "setDragCoefficient", "setCD", "setCx");
        }
        return forces;
    }

    // Keep the class lean: no 3-arg variant and no aeroHammer path.

    // ---- lifecycle: end -----------------------------------------------------
    @Override
    public void endSimulation(final SimulationStatus status, final SimulationException e) {
        if (dbg != null) dbg.note("Simulation ended");
    }

    // ---- helpers ------------------------------------------------------------
    private static boolean hasMethod(Object obj, String name) {
        try { obj.getClass().getMethod(name); return true; }
        catch (Throwable t) { return false; }
    }

    private static double clamp(double x, double lo, double hi) { return Math.min(Math.max(x, lo), hi); }
    private static double clamp01(double x) { return clamp(x, 0.0, 1.0); }
    private static double finiteOr(double x, double fb) { return Double.isFinite(x) ? x : fb; }

    private static double safeGet(final SimulationStatus st, final FlightDataType type, final double fallback) {
        try {
            final double v = st.getFlightDataBranch().getLast(type);
            return Double.isFinite(v) ? v : fallback;
        } catch (Throwable ignore) { return fallback; }
    }

    private static double safeTargetApogee(AirbrakeConfig cfg) {
        try {
            Method m = cfg.getClass().getMethod("getTargetApogee");
            Object v = m.invoke(cfg);
            if (v instanceof Number) return ((Number) v).doubleValue();
        } catch (Throwable ignore) { }
        return 0.0;
    }

    /**
     * Best-effort: write a user-visible "Airbrake Extension (%)".
     */
    private static void tryEmitExtensionToFlightData(final SimulationStatus st, final double deployFrac) {
        try {
            final Object branch = st.getFlightDataBranch();
            final Class<?> c = branch.getClass();
            for (String m : new String[]{"setUserData", "setCustomData", "putUserData"}) {
                try {
                    c.getMethod(m, String.class, double.class).invoke(branch, "Airbrake Extension (%)", deployFrac * 100.0);
                    return;
                } catch (NoSuchMethodException ignored) { }
            }
        } catch (Throwable ignore) { }
    }

    private static String tryGetSimName(final SimulationStatus status) {
        if (status == null) return null;
        try {
            Method m = status.getClass().getMethod("getSimulationName");
            Object v = m.invoke(status);
            if (v != null) return v.toString();
        } catch (Throwable ignore) { }
        try {
            Method m = status.getClass().getMethod("getSimulation");
            Object simObj = m.invoke(status);
            if (simObj != null) {
                try {
                    Method m2 = simObj.getClass().getMethod("getName");
                    Object v = m2.invoke(simObj);
                    if (v != null) return v.toString();
                } catch (Throwable ignore2) { }
            }
        } catch (Throwable ignore) { }
        return null;
    }

    private static boolean trySetDouble(final Object obj, final double val, final String... setters) {
        for (String s : setters) {
            try { obj.getClass().getMethod(s, double.class).invoke(obj, val); return true; }
            catch (Throwable ignore) {}
        }
        return false;
    }

    // Simple gate to mirror Waterloo’s “coast + speed” policy
    private boolean isExtensionAllowed(final SimulationStatus status) {
        final Coordinate v = status.getRocketVelocity();
        final double V = (v != null) ? Math.sqrt(Math.max(0, v.length2())) : 0.0;
        return coastLatched && V > 23.5 && currentAirbrakeDeployment > 1e-6;
    }

    // -------------------- Debug helpers --------------------
    private void setupDebug(final AirbrakeConfig cfg, final String simName) {
        try {
            if (cfg != null && cfg.isDebugEnabled()) {
                final String tag = (simName == null || simName.isBlank()) ? "sim" : simName.replaceAll("\\W+","_");
                dbg = new DebugTrace(true, cfg.isDbgWriteCsv(), cfg.isDbgShowConsole(), cfg.getDbgCsvDir(), tag);
                dbg.note("DebugTrace initialized");
            } else {
                dbg = new DebugTrace(false, false, false, "", null);
            }
        } catch (Throwable t) {
            dbg = null;
        }
    }
}
