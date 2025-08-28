package com.airbrakesplugin;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.AirDensity;

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
 * AirbrakeSimulationListener — coefficient-only injection (Waterloo structure)
 *
 * - Uses ApogeePredictorListener to estimate apogee (axis accel w/ gravity subtracted)
 * - Uses bang-bang AirbrakeController to set commanded endstate (extend / retract)
 * - preStep(): feed predictor, run controller, rate-limit actuator toward commandedDeploy
 * - postAerodynamicCalculation(): add ΔCd (and optional ΔCm) via reflection
 * - Keeps custom dynamic pressure path via AirDensity (compressibility-aware)
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // ---- config & sim objects ------------------------------------------------
    private final AirbrakeConfig config;
    private Rocket rocket;                // optional; not required for coeff updates
    private SimulationStatus sim;

    // ---- subsystems ----------------------------------------------------------
    private AirbrakeAerodynamics aerodynamics;     // ΔCd(M, deploy), optional ΔCm
    private com.airbrakesplugin.util.ApogeePredictorListener predictor;     // live OR-driven apogee predictor
    private AirbrakeController   controller;       // bang-bang controller

    // ---- geometry & limits (SI) ---------------------------------------------
    private double Sref = 0.0;             // m²
    private double Lref = 0.0;             // m
    private double maxRate = 4.0;          // fraction/s (actuator slew)

    // ---- state ---------------------------------------------------------------
    private double lastTime_s = 0.0;
    private double deploy     = 0.0;       // physical actuator state [0..1]
    private double commandedDeploy = 0.0;  // controller desired endstate (0 or 1)
    private double currentAirbrakeDeployment = 0.0;

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
        }
        catch (Throwable ignore) { 
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

        // Predictor: uses OpenRocket sim data; subtracts gravity; aligns along rocket axis
        log.debug("[Airbrakes] Initializing apogee predictor");
        this.predictor = new com.airbrakesplugin.util.ApogeePredictorListener();

        // Controller: simple bang-bang using predictor apogee
        double target = safeTargetApogee(config);
        this.commandedDeploy = 0.0; // start closed
        log.debug("[Airbrakes] Initializing controller with target apogee: {} m", target);
        this.controller = new AirbrakeController(target, predictor, new AirbrakeController.ControlContext() {
            @Override public void extend_airbrakes() { 
                commandedDeploy = 1.0; 
                log.debug("[Airbrakes] Controller commanding EXTEND");
            }
            @Override public void retract_airbrakes() { 
                commandedDeploy = 0.0; 
                log.debug("[Airbrakes] Controller commanding RETRACT");
            }
            @Override public void switch_altitude_back_to_pressure() { 
                log.debug("[Airbrakes] Controller switching altitude back to pressure (no-op)");
            }
        });

        // Config overrides (safe defaults already set above)
        Sref   = config.getReferenceArea();
        Lref   = config.getReferenceLength();
        maxRate = config.getMaxDeploymentRate();

        deploy     = 0.0;
        lastTime_s = finiteOr(status.getSimulationTime(), 0.0);

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

        // 1) Feed predictor with the live sim step (it reads V, orientation, altitude, etc.)
        try {
            log.debug("[Airbrakes] t={}: Updating predictor", t);
            predictor.postStep(status); // safe: we call it explicitly to keep everything in one listener
            double predictedApogee = predictor.getLatestPredictedApogee();
            log.debug("[Airbrakes] Predicted apogee: {} m", predictedApogee);
        } catch (Throwable ex) {
            // predictor is best-effort; don't fail the sim if something goes wrong
            log.debug("[Airbrakes] predictor.postStep error: {}", ex.toString());
        }

        // 2) Run controller (bang-bang on predicted apogee)
        try {
            log.debug("[Airbrakes] t={}: Running controller", t);
            double oldCommand = commandedDeploy;
            controller.update(status);
            if (oldCommand != commandedDeploy) {
                log.info("[Airbrakes] t={}: Controller changed command from {} to {}", 
                        t, oldCommand, commandedDeploy);
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
        
        if (Math.abs(deploy - oldDeploy) > 0.001) {
            log.debug("[Airbrakes] t={}: Deployment changed from {:.4f} to {:.4f} (commanded: {}, dmax: {:.4f})",
                    t, oldDeploy, deploy, commandedDeploy, dmax);
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
            log.debug("[Airbrakes] t={}: Skipping aero calcs due to near-zero velocity ({} m/s)", t, V);
            return base; // avoid near-zero noise
        }

        // Altitude & Mach (compute Mach from local a(alt) if needed)
        final double alt = safeGet(status, FlightDataType.TYPE_ALTITUDE, 0.0);

        double mach = safeGet(status, FlightDataType.TYPE_MACH_NUMBER, Double.NaN);
        if (!Double.isFinite(mach) || mach <= 0) {
            try {
                final double a = AirDensity.speedOfSoundISA(alt);
                if (a > 1e-6) mach = V / a;
                log.debug("[Airbrakes] t={}: Calculated Mach from V/a: {:.3f}", t, mach);
            } catch (Throwable ignore) { /* fallback below */ }
        }
        if (!Double.isFinite(mach) || mach <= 0) {
            try { 
                mach = AirDensity.machFromV(V, alt); 
                log.debug("[Airbrakes] t={}: Used utility to calculate Mach: {:.3f}", t, mach);
            }
            catch (Throwable ignore) { mach = 0.0; }
        }

        // Dynamic pressure (compressibility-aware via effective density)
        final double rhoEff = AirDensity.rhoForDynamicPressure(alt, mach); // includes qc/q_inc above M~0.3
        final double q = 0.5 * rhoEff * V * V;
        if (!(q > EPS_Q)) {
            log.debug("[Airbrakes] t={}: Skipping aero calcs due to insufficient dynamic pressure ({} Pa)", t, q);
            return base;
        }

        // Δcoeffs from tables (binary deploy handled inside AirbrakeAerodynamics if you made it so)
        double dCd_tab = aerodynamics.getIncrementalCd(mach, deployFrac);
        if (!Double.isFinite(dCd_tab) || Math.abs(dCd_tab) < 1e-12) {
            log.debug("[Airbrakes] t={}: Invalid or negligible ΔCd={} from tables", t, dCd_tab);
            return base;
        }

        double dCm_tab = 0.0;
        try {
            final double tmp = aerodynamics.getIncrementalCm(mach, deployFrac);
            if (Double.isFinite(tmp)) dCm_tab = tmp;
            log.debug("[Airbrakes] t={}: Retrieved ΔCm={} from tables", t, dCm_tab);
        } catch (Throwable ignore) { 
            log.debug("[Airbrakes] t={}: No ΔCm available, using 0", t);
        }

        // --- Force domain (apply ramp vs. very low-q) ---
        final double ramp = qRampScale(q);                 // 0..1
        final double dF   = ramp * (q * Sref * dCd_tab);   // N
        final double dM   = ramp * (q * Sref * Lref * dCm_tab); // N·m

        log.debug("[Airbrakes] t={}: alt={:.1f}m, V={:.1f}m/s, M={:.2f}, q={:.1f}Pa, deploy={:.2f}, ramp={:.2f}, dF={:.3f}N, dM={:.3f}Nm",
                t, alt, V, mach, q, deployFrac, ramp, dF, dM);

        // --- Back to coefficients for OR update ---
        final double inv_qS  = 1.0 / (q * Sref);
        final double inv_qSL = 1.0 / (q * Sref * Lref);

        final double dCd_eff = dF * inv_qS;     // == ramp * dCd_tab when valid
        final double dCm_eff = dM * inv_qSL;    // == ramp * dCm_tab when valid

        log.debug("[Airbrakes] t={}: Applying effective ΔCd={:.4f}, ΔCm={:.4f}", t, dCd_eff, dCm_eff);

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
            
            // Additional debugging info at sim end
            if (predictor != null) {
                log.info("[Airbrakes] Final predicted apogee: {} m", predictor.getLatestPredictedApogee());
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
            // Prefer a dedicated getter if present
            Method m = cfg.getClass().getMethod("getTargetApogee");
            Object v = m.invoke(cfg);
            if (v instanceof Number) return ((Number) v).doubleValue();
        } catch (Throwable ignore) { /* fall through */ }
        // Fallback to 0 if not configured
        return 0.0;
    }

    /**
     * Adds delta to a double property on 'obj' using common setter spellings.
     * Returns true if applied.
     */
    private static boolean addToDoubleProperty(final Object obj, final double value, final String... setterCandidates) {
        if (!Double.isFinite(value)) return true;
        // Try any direct setter first.
        for (String name : setterCandidates) {
            try {
                Method m = obj.getClass().getMethod(name, double.class);
                m.invoke(obj, value);
                return true;
            } catch (NoSuchMethodException ignored) { /* try next */ }
            catch (Throwable t) { /* try next */ }
        }
        // Fallback: read-modify-write if a getter exists (rare with OR internals)
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

    /**
     * Best-effort: write a user-visible "Airbrake Extension (%)".
     * On some OR versions there's no API for dynamic columns; this silently no-ops.
     */
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
        // linear ramp 0→1 between Q_RAMP_START_PA and Q_RAMP_FULL_PA
        if (!(q > 0)) return 0.0;
        final double t = (q - Q_RAMP_START_PA) / (Q_RAMP_FULL_PA - Q_RAMP_START_PA);
        return Math.max(0.0, Math.min(1.0, t));
    }
}
