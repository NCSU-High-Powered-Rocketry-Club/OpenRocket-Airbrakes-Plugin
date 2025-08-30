package com.airbrakesplugin;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.ApogeePredictor;

import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;

import java.lang.reflect.Method;
import java.util.Objects;

/**
 * AirbrakeSimulationListener — coefficient-only injection (Waterloo structure)
 *
 * Python-parity wiring:
 *  - Compute vertical accel INCLUDING g via dv/dt on WORLD_Z; call predictor.update(...) ONLY during coast.
 *  - The predictor subtracts g internally in LUT building (parity with Python).
 *  - Controller reads predictor.getPredictionIfReady() for bang-bang.
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // ---- config & sim objects ------------------------------------------------
    private final AirbrakeConfig config;
    private Rocket rocket;                // optional; not required for coeff updates
    private SimulationStatus sim;

    // ---- subsystems ----------------------------------------------------------
    private AirbrakeAerodynamics aerodynamics;     // ΔCd(M, deploy), optional ΔCm
    private ApogeePredictor      predictor;        // Python-parity predictor (no internal gating)
    private AirbrakeController   controller;       // bang-bang controller

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
    private static final double THRUST_EPSILON_N     = 1.0; // thrust≈0 gate
    private static final int    THRUST_CONSEC_HITS   = 2;
    private static final double ACCEL_MINUS_G_THRESH = 1.5; // m/s^2, small decel gate
    private static final int    ACCEL_CONSEC_HITS    = 2;
    private static final double COAST_TIME_FALLBACKS = 1.0; // s

    // ---- Q-ramp to avoid low-q blowups --------------------------------------
    private static final double Q_RAMP_START_PA = 5.0;   // start applying effect ~5 Pa
    private static final double Q_RAMP_FULL_PA  = 50.0;  // full effect by ~50 Pa
    private static final double EPS_Q           = 1e-6;

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
            log.debug("[Airbrakes] Retrieved rocket reference from simulation status");
        } catch (Throwable ignore) {
            log.debug("[Airbrakes] Could not retrieve rocket from simulation status");
        }

        // Load aero tables
        try {
            log.debug("[Airbrakes] Loading aerodynamics data from: {}", config.getCfdDataFilePath());
            this.aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
            log.debug("[Airbrakes] Aerodynamics data loaded successfully");
        } catch (Exception e) {
            log.error("Failed to load airbrake CFD data from {}: {}", config.getCfdDataFilePath(), e.toString());
            throw new SimulationException("Airbrake CFD data failed to load", e);
        }

        // Predictor
        this.predictor = ApogeePredictor.defaultConfig();

        // Controller
        double target = safeTargetApogee(config);
        this.commandedDeploy = 0.0; // start closed
        log.debug("[Airbrakes] Initializing controller with target apogee: {} m", target);
        this.controller = new AirbrakeController(target, predictor, new AirbrakeController.ControlContext() {
            @Override public void extend_airbrakes() { commandedDeploy = 1.0; log.debug("[Airbrakes] EXTEND"); }
            @Override public void retract_airbrakes() { commandedDeploy = 0.0; log.debug("[Airbrakes] RETRACT"); }
            @Override public void switch_altitude_back_to_pressure() { /* no-op */ }
        });

        // Config overrides
        Sref   = config.getReferenceArea();
        Lref   = config.getReferenceLength();
        maxRate = config.getMaxDeploymentRate();

        deploy     = 0.0;
        lastTime_s = finiteOr(status.getSimulationTime(), 0.0);
        lastVaxis  = null;
        coastLatched = false;
        thrustZeroHits = 0;
        accelGateHits  = 0;

        log.info("[Airbrakes] init: Sref={} m^2, Lref={} m, maxRate={} frac/s, target={} m",
                Sref, Lref, maxRate, target);
    }

    // ---- lifecycle: step state update (predictor, controller, actuator) -----
    @Override
    public boolean preStep(final SimulationStatus status) {
        if (aerodynamics == null || controller == null || predictor == null) {
            log.debug("[Airbrakes] preStep: skipping due to uninitialized components");
            return true;
        }

        final double t  = finiteOr(status.getSimulationTime(), lastTime_s);
        final double dt = Math.max(0.0, t - lastTime_s);
        lastTime_s = t;
        if (dt <= 0.0) return true;

        // ------------------- WORLD-Z VERTICAL KINEMATICS (UP-POSITIVE) --------
        final Coordinate vWorld = status.getRocketVelocity();
        final double vAxis      = (vWorld != null) ? vWorld.z : 0.0;  // vertical up
        final double alt        = safeGet(status, FlightDataType.TYPE_ALTITUDE, 0.0);

        double aNetAxis = Double.NaN;
        if (lastVaxis != null) aNetAxis = (vAxis - lastVaxis) / Math.max(1e-6, dt);
        lastVaxis = vAxis;

        // For gating only (don’t subtract g when calling predictor.update)
        final double gAxis   = -G;                                 // gravity along +Z up is negative
        final double aMinusG = Double.isFinite(aNetAxis) ? (aNetAxis - gAxis) : Double.NaN;
        // ----------------------------------------------------------------------

        // --------------------- COAST GATING (listener-level) ------------------
        boolean thrustGate = false, accelGate = false, timeGate = false;
        final double thrust = safeGet(status, FlightDataType.TYPE_THRUST_FORCE, Double.NaN);
        if (Double.isFinite(thrust) && thrust <= THRUST_EPSILON_N) {
            thrustZeroHits++;
            thrustGate = (thrustZeroHits >= THRUST_CONSEC_HITS);
        } else {
            thrustZeroHits = 0;
        }

        // **FIX 1**: remove velocity sign from accel gate — let samples in as soon as decel is small
        if (Double.isFinite(aMinusG) && aMinusG <= ACCEL_MINUS_G_THRESH) {
            accelGateHits++;
            accelGate = (accelGateHits >= ACCEL_CONSEC_HITS);
        } else {
            accelGateHits = 0;
        }

        timeGate = (t >= COAST_TIME_FALLBACKS);

        if (!coastLatched && (thrustGate || accelGate || timeGate)) {
            coastLatched = true;
            log.debug("[Airbrakes] t={}s: COAST latched (thrustGate={}, accelGate={}, timeGate={}), v0={} m/s",
                    String.format("%.3f", t), thrustGate, accelGate, timeGate, String.format("%.2f", vAxis));
        }

        // -------------------- FEED PREDICTOR (Python parity) ------------------
        // **FIX 2**: remove velocity sign from predictor feed — collect as soon as coast is latched
        if (coastLatched && Double.isFinite(aNetAxis)) {
            predictor.update(aNetAxis, dt, alt, vAxis); // accel includes g (parity with Python)
            final Double ap = predictor.getPredictionIfReady();
            if (ap != null) {
                log.debug("[Airbrakes] t={}s: Predictor apogee ready: {} m",
                        String.format("%.3f", t), String.format("%.2f", ap));
            } else {
                log.debug("[Airbrakes] t={}s: Predictor not converged yet (samples={})",
                        String.format("%.3f", t), predictor.sampleCount());
            }
        }

        // --------------- Controller: bang-bang using predictor ----------------
        try {
            double before = commandedDeploy;
            controller.update(status);
            if (before != commandedDeploy) {
                log.info("[Airbrakes] t={}s: Controller changed command {} -> {}",
                        String.format("%.3f", t), before, commandedDeploy);
            }
        } catch (Throwable ex) {
            log.debug("[Airbrakes] controller.update error: {}", ex.toString());
        }

        // 3) Evolve physical actuator toward commanded endstate at maxRate
        final double dmax  = maxRate * dt;        // max change this step
        final double lo    = Math.max(0.0, commandedDeploy - dmax);
        final double hi    = Math.min(1.0, commandedDeploy + dmax);
        double oldDeploy = deploy;
        deploy = clamp01(clamp(deploy, lo, hi));
        if (Math.abs(deploy - oldDeploy) > 1e-6) {
            log.debug("[Airbrakes] t={}s: Deployment {} -> {} (cmd={}, dmax={})",
                    String.format("%.3f", t),
                    String.format("%.4f", oldDeploy),
                    String.format("%.4f", deploy),
                    String.format("%.2f", commandedDeploy),
                    String.format("%.4f", dmax));
        }

        currentAirbrakeDeployment = deploy;
        tryEmitExtensionToFlightData(status, deploy);

        return true;
    }

    // ---- single-point coefficient injection (Waterloo pattern) -------------
    @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status, final AerodynamicForces base) throws SimulationException {
        if (aerodynamics == null || base == null || status == null) {
            log.debug("[Airbrakes] postAerodynamicCalculation: skipping due to null components");
            return base;
        }

        final double deployFrac = clamp01(this.deploy);
        if (deployFrac <= 1e-6) return base;

        final double t = status.getSimulationTime();

        // Velocity magnitude
        final Coordinate vel = status.getRocketVelocity();
        final double V = (vel != null) ? vel.length() : 0.0;
        if (!(V > 0.1)) {
            log.debug("[Airbrakes] t={}s: Skipping aero calcs due to near-zero velocity ({} m/s)", String.format("%.3f", t), V);
            return base; // avoid near-zero noise
        }

        // Altitude & Mach (compute Mach from local a(alt) if needed)
        final double alt = safeGet(status, FlightDataType.TYPE_ALTITUDE, 0.0);

        double mach = safeGet(status, FlightDataType.TYPE_MACH_NUMBER, Double.NaN);
        if (!Double.isFinite(mach) || mach <= 0) {
            try {
                final double a = AirDensity.speedOfSoundISA(alt);
                if (a > 1e-6) mach = V / a;
                log.debug("[Airbrakes] t={}s: Calculated Mach from V/a: {}", String.format("%.3f", t), String.format("%.3f", mach));
            } catch (Throwable ignore) { /* fallback below */ }
        }
        if (!Double.isFinite(mach) || mach <= 0) {
            try {
                mach = AirDensity.machFromV(V, alt);
                log.debug("[Airbrakes] t={}s: Used utility to calculate Mach: {}", String.format("%.3f", t), String.format("%.3f", mach));
            } catch (Throwable ignore) { mach = 0.0; }
        }

        // Dynamic pressure (compressibility-aware via effective density)
        final double rhoEff = AirDensity.rhoForDynamicPressure(alt, mach); // includes qc/q_inc above M~0.3
        final double q = 0.5 * rhoEff * V * V;
        if (!(q > EPS_Q)) {
            log.debug("[Airbrakes] t={}s: Skipping aero calcs due to low dynamic pressure ({} Pa)", String.format("%.3f", t), q);
            return base;
        }

        // Δcoeffs from tables
        double dCd_tab = aerodynamics.getIncrementalCd(mach, deployFrac);
        if (!Double.isFinite(dCd_tab) || Math.abs(dCd_tab) < 1e-12) {
            log.debug("[Airbrakes] t={}s: Invalid or negligible ΔCd={} from tables", String.format("%.3f", t), dCd_tab);
            return base;
        }

        double dCm_tab = 0.0;
        try {
            final double tmp = aerodynamics.getIncrementalCm(mach, deployFrac);
            if (Double.isFinite(tmp)) dCm_tab = tmp;
            log.debug("[Airbrakes] t={}s: Retrieved ΔCm={} from tables", String.format("%.3f", t), dCm_tab);
        } catch (Throwable ignore) {
            log.debug("[Airbrakes] t={}s: No ΔCm available, using 0", String.format("%.3f", t));
        }

        // --- Force domain (apply ramp vs. very low-q) ---
        final double ramp = qRampScale(q);                 // 0..1
        final double dF   = ramp * (q * Sref * dCd_tab);   // N
        final double dM   = ramp * (q * Sref * Lref * dCm_tab); // N·m

        log.debug("[Airbrakes] t={}s: alt={} m, V={} m/s, M={}, q={} Pa, deploy={}, ramp={}, dF={} N, dM={} N·m",
                String.format("%.3f", t),
                String.format("%.1f", alt),
                String.format("%.1f", V),
                String.format("%.2f", mach),
                String.format("%.1f", q),
                String.format("%.2f", deployFrac),
                String.format("%.2f", ramp),
                String.format("%.3f", dF),
                String.format("%.3f", dM));

        // --- Back to coefficients for OR update ---
        final double inv_qS  = 1.0 / (q * Sref);
        final double inv_qSL = 1.0 / (q * Sref * Lref);

        final double dCd_eff = dF * inv_qS;     // == ramp * dCd_tab when valid
        final double dCm_eff = dM * inv_qSL;    // == ramp * dCm_tab when valid

        log.debug("[Airbrakes] t={}s: Applying effective ΔCd={}, ΔCm={}",
                String.format("%.3f", t), String.format("%.4f", dCd_eff), String.format("%.4f", dCm_eff));

        // ---- Apply ΔC* via reflection on AerodynamicForces ----
        final boolean cdApplied = addToDoubleProperty(base, dCd_eff,
                "setDragCoefficient", "setCD", "setCx", "setAxialForceCoefficient", "setCDAxial");

        final boolean cmApplied = (Math.abs(dCm_eff) < 1e-12) ? true :
                addToDoubleProperty(base, dCm_eff,
                        "setPitchingMomentCoefficient", "setCM", "setCm", "setCMy", "setCmPitch");

        if (!cdApplied) {
            log.debug("[Airbrakes] No drag-coefficient setter found on AerodynamicForces; leaving base unchanged.");
        }
        if (!cmApplied) {
            log.debug("[Airbrakes] No pitching-moment-coefficient setter found; ΔCm skipped.");
        }

        return base;
    }

    // ---- lifecycle: end -----------------------------------------------------
    @Override
    public void endSimulation(final SimulationStatus status, final SimulationException e) {
        if (e != null) {
            log.error("[Airbrakes] Simulation ended with error", e);
        } else {
            log.info("[Airbrakes] Simulation complete – final deploy={}, commanded={}", deploy, commandedDeploy);
            if (predictor != null) {
                final Double ap = predictor.getPredictionIfReady();
                log.info("[Airbrakes] Final predicted apogee: {}", (ap != null) ? ap : "n/a");
            }
            double altitude = safeGet(status, FlightDataType.TYPE_ALTITUDE, -1);
            log.info("[Airbrakes] Final altitude: {} m", altitude);
        }
    }

    // ---- helpers ------------------------------------------------------------
    private static double clamp(double x, double lo, double hi) { return Math.min(Math.max(x, lo), hi); }
    private static double clamp01(double x) { return clamp(x, 0.0, 1.0); }
    private static double finiteOr(double x, double fb) { return Double.isFinite(x) ? x : fb; }

    private static double safeGet(final SimulationStatus st, final FlightDataType type, final double fallback) {
        try {
            final double v = st.getFlightDataBranch().getLast(type);
            return Double.isFinite(v) ? v : fallback;
        } catch (Throwable ignore) {
            return fallback;
        }
    }

    private static double safeTargetApogee(AirbrakeConfig cfg) {
        try {
            Method m = cfg.getClass().getMethod("getTargetApogee");
            Object v = m.invoke(cfg);
            if (v instanceof Number) return ((Number) v).doubleValue();
        } catch (Throwable ignore) { /* fall through */ }
        return 0.0;
    }

    private static boolean addToDoubleProperty(final Object obj, final double value, final String... setterCandidates) {
        if (!Double.isFinite(value)) return true;
        for (String name : setterCandidates) {
            try {
                Method m = obj.getClass().getMethod(name, double.class);
                m.invoke(obj, value);
                return true;
            } catch (NoSuchMethodException ignored) { /* try next */ }
            catch (Throwable t) { /* try next */ }
        }
        Double current = tryGetAnyDouble(obj,
                "getDragCoefficient", "getCD", "getCx", "getAxialForceCoefficient", "getCDAxial",
                "getPitchingMomentCoefficient", "getCM", "getCm", "getCMy", "getCmPitch"
        );
        if (current != null) {
            double updated = current + value;
            for (String name : setterCandidates) {
                try {
                    Method m = obj.getClass().getMethod(name, double.class);
                    m.invoke(obj, updated);
                    return true;
                } catch (NoSuchMethodException ignored) { /* try next */ }
                catch (Throwable t) { /* try next */ }
            }
        }
        return false;
    }

    private static Double tryGetAnyDouble(final Object obj, final String... getters) {
        for (String g : getters) {
            try {
                Method m = obj.getClass().getMethod(g);
                Object v = m.invoke(obj);
                if (v instanceof Number) return ((Number) v).doubleValue();
            } catch (NoSuchMethodException ignored) { /* try next */ }
            catch (Throwable t) { /* try next */ }
        }
        return null;
    }

    private static void tryEmitExtensionToFlightData(final SimulationStatus st, final double deployFrac) {
        try {
            final Object branch = st.getFlightDataBranch();
            final Class<?> c = branch.getClass();
            for (String m : new String[]{"setUserData", "setCustomData", "putUserData"}) {
                try {
                    c.getMethod(m, String.class, double.class).invoke(branch, "Airbrake Extension (%)", deployFrac * 100.0);
                    return;
                } catch (NoSuchMethodException ignored) { /* try next */ }
            }
        } catch (Throwable ignore) { /* optional feature */ }
    }

    private static double qRampScale(double q) {
        if (!(q > 0)) return 0.0;
        final double t = (q - Q_RAMP_START_PA) / (Q_RAMP_FULL_PA - Q_RAMP_START_PA);
        return Math.max(0.0, Math.min(1.0, t));
    }
}
