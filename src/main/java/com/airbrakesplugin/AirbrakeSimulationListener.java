package com.airbrakesplugin;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
 * - preStep(): evolve actuator (rate-limited)
 * - postProcessAerodynamicForces(): add ΔCd (and optional ΔCm) to "base" via reflection
 * - keeps AirDensity.rhoForDynamicPressure(alt, Mach)
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // ---- config & sim objects ------------------------------------------------
    private final AirbrakeConfig config;
    private Rocket rocket;                // optional; not required for coeff updates
    private SimulationStatus sim;

    // ---- subsystems ----------------------------------------------------------
    private AirbrakeAerodynamics aerodynamics;  // ΔCd(M, deploy), optional ΔCm
    private AirbrakeController   controller;    // commanded deployment [0..1]

    // ---- geometry & limits (SI) ---------------------------------------------
    private double Sref = 0.008706;  // m^2, default to 8706 mm^2
    private double Lref = 0.1016;    // m,   default to 4 in
    private double maxRate = 1.0;    // fraction/s

    // ---- state ---------------------------------------------------------------
    private double lastTime_s = 0.0;
    private double deploy     = 0.0;   // physical actuator state [0..1]
    private double currentAirbrakeDeployment = 0.0;

// Add near top of the class with other constants:
    private static final double Q_RAMP_START_PA = 5.0;   // start applying effect ~5 Pa
    private static final double Q_RAMP_FULL_PA  = 50.0;  // full effect by ~50 Pa
    private static final double EPS_Q           = 1e-6;


    public AirbrakeSimulationListener(final AirbrakeConfig cfg, final Rocket rkt) {
        this.config = Objects.requireNonNull(cfg, "AirbrakeConfig must not be null");
        this.rocket = rkt;
    }

    public double getCurrentAirbrakeDeployment() { return currentAirbrakeDeployment; }
    public boolean isAirbrakeCurrentlyDeployed() { return currentAirbrakeDeployment > 1e-6; }

    // ---- lifecycle: begin ----------------------------------------------------
    @Override
    public void startSimulation(final SimulationStatus status) throws SimulationException {
        this.sim = status;
        try { this.rocket = (Rocket) status.getClass().getMethod("getRocket").invoke(status); }
        catch (Throwable ignore) { /* ok */ }

        // Load aero tables
        try {
            this.aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception e) {
            log.error("Failed to load airbrake CFD data from {}: {}", config.getCfdDataFilePath(), e.toString());
            throw new SimulationException("Airbrake CFD data failed to load", e);
        }

        // Controller
        this.controller = new AirbrakeController(config);
        this.controller.setContext(new AirbrakeController.ControlContext() {
            @Override public void extend_airbrakes() {/* no-op */}
            @Override public void retract_airbrakes() {/* no-op */}
            @Override public void switch_altitude_back_to_pressure() {/* no-op */}
        });

        // Config overrides (safe defaults already set above)
        if (config.getReferenceArea()   > 0) Sref = config.getReferenceArea();
        if (config.getReferenceLength() > 0) Lref = config.getReferenceLength();
        if (config.getMaxDeploymentRate() > 0) maxRate = config.getMaxDeploymentRate();

        deploy     = 0.0;
        lastTime_s = finiteOr(status.getSimulationTime(), 0.0);

        log.info("[Airbrakes] init: Sref={} m^2, Lref={} m, maxRate={} frac/s", Sref, Lref, maxRate);
    }

    // ---- lifecycle: step state update (actuator only) -----------------------
    @Override
    public boolean preStep(final SimulationStatus status) {
        if (aerodynamics == null || controller == null) return true;

        final double t  = finiteOr(status.getSimulationTime(), lastTime_s);
        final double dt = Math.max(0.0, t - lastTime_s);
        lastTime_s = t;
        if (dt <= 0.0) return true;

        // Inputs for controller
        final double alt  = safeGet(status, FlightDataType.TYPE_ALTITUDE,    0.0);
        final double mach = safeGet(status, FlightDataType.TYPE_MACH_NUMBER, 0.0);
        final Coordinate v = status.getRocketVelocity();
        final double vz    = (v != null && Double.isFinite(v.z)) ? v.z : 0.0;

        double az = Double.NaN;
        try {
            final double accZ = status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_Z);
            if (Double.isFinite(accZ)) az = accZ;
        } catch (Throwable ignore) { /* ok */ }

        // Command & rate limit
        final double u_cmd = controller.getCommandedDeployment(alt, vz, az, mach, deploy, dt, status);
        final double dmax  = maxRate * dt;
        deploy = clamp01(clamp(u_cmd, deploy - dmax, deploy + dmax));

        currentAirbrakeDeployment = deploy;
        tryEmitExtensionToFlightData(status, deploy);

        return true;
    }

    // ---- single-point coefficient injection (Waterloo pattern) -------------
    @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status, final AerodynamicForces base) throws SimulationException {
        if (aerodynamics == null || base == null || status == null) return base;

        final double deployFrac = clamp01(this.deploy);
        if (deployFrac <= 1e-6) return base;

        // Velocity magnitude
        final Coordinate vel = status.getRocketVelocity();
        final double V = (vel != null) ? vel.length() : 0.0;
        if (!(V > 0.1)) return base; // avoid near-zero noise

        // Altitude & Mach
        final double alt = safeGet(status, FlightDataType.TYPE_ALTITUDE, 0.0);
        double mach = safeGet(status, FlightDataType.TYPE_MACH_NUMBER, Double.NaN);
        if (!Double.isFinite(mach) || mach <= 0) {
            // Fallback: compute Mach from V & altitude using your AirDensity
            mach = com.airbrakesplugin.util.AirDensity.machFromV(V, alt);
        }

        // Dynamic pressure (compressibility-aware). Uses qc = Pt - p for M > 0.3, else ½ρV².
        final double q = com.airbrakesplugin.util.AirDensity.dynamicPressureFromMach(alt, mach, V);
        if (!(q > EPS_Q)) return base;

        // Δcoeffs from tables (binary deploy handled inside AirbrakeAerodynamics)
        double dCd_tab = aerodynamics.getIncrementalCd(mach, deployFrac);
        if (!Double.isFinite(dCd_tab) || Math.abs(dCd_tab) < 1e-12) return base;

        double dCm_tab = 0.0;
        try {
            final double tmp = aerodynamics.getIncrementalCm(mach, deployFrac);
            if (Double.isFinite(tmp)) dCm_tab = tmp;
        } catch (Throwable ignore) { /* leave zero */ }

        // --- Force domain (apply ramp vs. very low-q) ---
        final double ramp = qRampScale(q);                 // 0..1
        final double dF   = ramp * (q * Sref * dCd_tab);   // N
        final double dM   = ramp * (q * Sref * Lref * dCm_tab); // N·m

        // --- Back to coefficients for OR update ---
        final double inv_qS  = (q > EPS_Q && Sref > 0.0) ? (1.0 / (q * Sref)) : 0.0;
        final double inv_qSL = (q > EPS_Q && Sref > 0.0 && Lref > 0.0) ? (1.0 / (q * Sref * Lref)) : 0.0;

        final double dCd_eff = dF * inv_qS;     // == ramp * dCd_tab when valid
        final double dCm_eff = dM * inv_qSL;    // == ramp * dCm_tab when valid

        // ---- Apply ΔC* via reflection on AerodynamicForces ----
        final boolean cdApplied = addToDoubleProperty(
                base, dCd_eff,
                "setDragCoefficient", "setCD", "setCx", "setAxialForceCoefficient", "setCDAxial"
        );

        final boolean cmApplied = (Math.abs(dCm_eff) < 1e-12) ? true : addToDoubleProperty(base, dCm_eff,"setPitchingMomentCoefficient", "setCM", "setCm", "setCMy", "setCmPitch");

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
        if (e != null) log.error("[Airbrakes] Simulation ended with error", e);
        else           log.info("[Airbrakes] Simulation complete – final deploy={}", deploy);
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

    /**
     * Adds delta to a double property on 'obj' using common getter/setter spellings.
     * Returns true if applied.
     */
    private static boolean addToDoubleProperty(final Object obj, final double delta, final String... setterCandidates) {
        if (Math.abs(delta) < 1e-15) return true;
        // Try to find a paired getter for read-modify-write. If none, try "addXxx(double)" style.
        // 1) Look for any getter that returns double and has "get.*Coeff" or short names (CD, CM, Cx...)
        Double current = tryGetAnyDouble(obj,
                "getDragCoefficient", "getCD", "getCx", "getAxialForceCoefficient", "getCDAxial",
                "getPitchingMomentCoefficient", "getCM", "getCm", "getCMy", "getCmPitch"
        );
        if (current != null) {
            final double updated = current + delta;
            for (String name : setterCandidates) {
                try {
                    Method m = obj.getClass().getMethod(name, double.class);
                    m.invoke(obj, updated);
                    return true;
                } catch (NoSuchMethodException ignored) { /* try next */ }
                catch (Throwable t) { /* try next */ }
            }
        }
        // 2) Try direct adder (e.g., addDragCoefficient(double))
        for (String set : setterCandidates) {
            final String addName = set.replaceFirst("^set", "add");
            try {
                Method m = obj.getClass().getMethod(addName, double.class);
                m.invoke(obj, delta);
                return true;
            } catch (NoSuchMethodException ignored) { /* try next */ }
            catch (Throwable t) { /* try next */ }
        }
        return false;
    }

    private static Double tryGetAnyDouble(final Object obj, final String... getters) {
        for (String g : getters) {
            try {
                Method m = obj.getClass().getMethod(g);
                if (m.getReturnType() == double.class || m.getReturnType() == Double.TYPE) {
                    Object v = m.invoke(obj);
                    if (v instanceof Double) return (Double) v;
                    if (v != null && v.getClass() == double.class) return (Double) v;
                    if (v == null) continue;
                    // handle primitive double via reflection boxing
                    return ((Number) v).doubleValue();
                }
            } catch (NoSuchMethodException ignored) { /* try next */ }
            catch (Throwable t) { /* try next */ }
        }
        return null;
    }

    /**
     * Best-effort: write a user-visible “Airbrake Extension (%)”.
     * On some OR versions there’s no API for dynamic columns; this silently no-ops.
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
