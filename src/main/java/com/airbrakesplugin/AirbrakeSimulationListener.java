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

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.Objects;

/**                 
 * Air-brake 6-DoF simulation listener – OpenRocket 23.09
 *
 * Injects incremental aerodynamic effects from airbrakes based on CFD tables.
 * Uses "force/moment injection" to avoid coefficient API limitations across builds.
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // ── config & sim objects ───────────────────────────────────────────────
    private final AirbrakeConfig config;
    private final Rocket rocket;

    // ── subsystems ─────────────────────────────────────────────────────────
    private AirbrakeAerodynamics aerodynamics;   // provides dCd, dCm = f(Mach, deploy)
    private AirbrakeController   controller;     // returns commanded deployment ∈ [0..1]

    // Mirror of the actuator state for external checks/UI (0..1)
    private double currentAirbrakeDeployment = 0.0;

    // Convenience boolean: true when any non-trivial deployment is present
    public boolean isAirbrakeCurrentlyDeployed() {
        return currentAirbrakeDeployment > 1e-6;
    }

    // Optional getter if other classes prefer the numeric value
    public double getCurrentAirbrakeDeployment() {
        return currentAirbrakeDeployment;
    }

    // ── geometry & limits (SI units) ───────────────────────────────────────
    private double Sref;      // m^2
    private double Lref;      // m
    private double maxRate;   // fraction/s (0.5 = 50%/s)

    // ── state ──────────────────────────────────────────────────────────────
    private double lastTime_s = 0.0;
    private double deploy     = 0.0;   // current physical deployment ∈ [0..1]
    
    // ── constants ─────────────────────────────────────────────────────────
    private static final double DEFAULT_MAX_RATE_FRAC_PER_S = 1.0; // fallback if config is 0/NaN

    // ── ctor ───────────────────────────────────────────────────────────────
    public AirbrakeSimulationListener(final AirbrakeConfig cfg, final Rocket rkt) {
        this.config = Objects.requireNonNull(cfg, "AirbrakeConfig must not be null");
        this.rocket = rkt;
    }

    // ── life-cycle: begin ──────────────────────────────────────────────────
    @Override
    public void startSimulation(final SimulationStatus status) throws SimulationException {
        // 0) geometry & limits
        this.maxRate = DEFAULT_MAX_RATE_FRAC_PER_S;
        // 1) subsystems
        try {
            this.aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception e) {
            log.error("Failed to load airbrake CFD data from {}: {}", config.getCfdDataFilePath(), e.toString());
            throw new SimulationException("Airbrake CFD data failed to load", e);
        }
        this.controller = new AirbrakeController(config);

        // Wire context callbacks (optional but makes your external "tell it to extend" hooks real)
        this.controller.setContext(new AirbrakeController.ControlContext() {
            @Override public void extend_airbrakes() {
                // no-op here; the actuator follows getCommandedDeployment's return value
                // (you could set a flag to bias the command if you want)
            }
            @Override public void retract_airbrakes() {
                // same note as above
            }
            @Override public void switch_altitude_back_to_pressure() {
                // If you have a pressure-altitude source that needs switching, do it here via reflection
                // or by calling your own hook.
            }
        });

        // 2) state init
        this.deploy     = 0.0;
        this.lastTime_s = finiteOr(status.getSimulationTime(), 0.0);

        log.info("Airbrake listener initialised: Sref={} m^2, Lref={} m, maxRate={} frac/s", Sref, Lref, maxRate);
    }

    // ── life-cycle: pre step (update actuator state) ───────────────────────
    @Override
    public boolean preStep(final SimulationStatus status) {
        if (aerodynamics == null || controller == null) return true;

        // 0) dt guard
        final double t  = finiteOr(status.getSimulationTime(), lastTime_s);
        final double dt = Math.max(0.0, t - lastTime_s);
        lastTime_s = t;
        if (dt <= 0.0) {
            return true;
        }

        // 1) gather minimal state for controller
        final double alt  = finiteOr(status.getRocketPosition().z, 0.0);
        final double vz   = finiteOr(status.getRocketVelocity().z, 0.0);

        double mach = 0.0;
        try {
            mach = status.getFlightDataBranch().getLast(FlightDataType.TYPE_MACH_NUMBER);
        } catch (Throwable ignore) { /* leave 0.0 */ }

        double az = Double.NaN; // prefer OR-provided accel; controller will dv/dt fallback if NaN
        try {
            final double acc_xy = status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_XY);
            final double acc_z  = status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_Z);

            final Coordinate acc = new Coordinate(acc_xy, 0.0, acc_z);

            if (acc != null && Double.isFinite(acc.z)) az = acc.z;
        } catch (Throwable ignore) { /* ok */ }

        // (optional) central gate using OR's current apogee vs threshold
        // if (!isAirbrakeAllowedToDeploy(status)) {
        //     // Optionally keep predictor clean / auto-stow here.
        //     return true; 
        // }

        // 2) commanded deployment (use ACTUAL actuator state 'deploy')
        double u_cmd = controller.getCommandedDeployment(alt, vz, az, mach, deploy, dt, status);

        // 3) actuator rate limit
        final double dmax  = maxRate * dt;                 // fraction this step
        final double u_new = clamp01(clamp(u_cmd, deploy - dmax, deploy + dmax));

        if (log.isDebugEnabled()) {
            log.debug("preStep t={}: dt={}, alt={}, vz={}, M={}, deploy={} -> cmd={} -> new={}",
                    t, dt, alt, vz, mach, deploy, u_cmd, u_new);
        }

        deploy = u_new;
        currentAirbrakeDeployment = deploy;   // keep mirror in sync
        return true; // continue sim
    }

    // ── force/moment injection ─────────────────────────────────────────────
    // ── force/moment injection (OpenRocket 23.x signature) ─────────────────────────
@Override
public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status,
                                                    final AerodynamicForces base)
        throws SimulationException {
    // Guards
    if (aerodynamics == null || rocket == null || status == null || base == null) return base;
    final double deployFrac = clamp01(this.deploy);
    if (deployFrac <= 1e-6) return base;

    // Kinematics
    final Coordinate vel = status.getRocketVelocity();
    final Coordinate pos = status.getRocketPosition();
    final double V   = (vel != null) ? vel.length() : 0.0;
    final double alt = (pos != null) ? pos.z : 0.0;
    if (!Double.isFinite(V) || V < 1e-6) return base;

    // Density (ISA) and Mach
    double rho = 1.225;
    try { rho = com.airbrakesplugin.util.AirDensity.rhoISA(alt); }
    catch (Throwable ignore) {
        try { rho = com.airbrakesplugin.util.AirDensity.rhoISA(alt); }
        catch (Throwable ignore2) { /* keep sea-level default */ }
    }
    double M = 0.0;
    try {
        final double m = status.getFlightDataBranch().getLast(FlightDataType.TYPE_MACH_NUMBER);
        if (Double.isFinite(m)) M = m;
    } catch (Throwable ignore) { /* M=0 */ }

    // Interpolated coefficient increments — table may be fraction [0..1] or percent [0..100]
    double dCd = aerodynamics.getIncrementalCd(M, deployFrac);
    double dCm = aerodynamics.getIncrementalCm(M, deployFrac);
    if ((Math.abs(dCd) + Math.abs(dCm)) < 1e-12 && deployFrac > 1e-3) {
        dCd = aerodynamics.getIncrementalCd(M, 100.0 * deployFrac);
        dCm = aerodynamics.getIncrementalCm(M, 100.0 * deployFrac);
    }
    if (!Double.isFinite(dCd) && !Double.isFinite(dCm)) return base;

    // Dynamic pressure and increments
    final double q = 0.5 * rho * V * V;

    // Incremental drag: opposite to velocity
    Coordinate dF = Coordinate.NUL;
    if (dCd != 0.0) {
        final Coordinate vhat = new Coordinate(vel.x / V, vel.y / V, vel.z / V);
        final double mag = q * Sref * dCd;
        dF = new Coordinate(-mag * vhat.x, -mag * vhat.y, -mag * vhat.z);
    }

    // Incremental pitching moment about +Y (OpenRocket: X fore-aft, Y pitch, Z up)
    Coordinate dM = Coordinate.NUL;
    if (dCm != 0.0) {
        final double dMmag = dCm * q * Sref * Lref;
        dM = new Coordinate(0.0, dMmag, 0.0);
    }

    if (dF == Coordinate.NUL && dM == Coordinate.NUL) return base;

    // Try to return a NEW AerodynamicForces(F, M, CP)
    try {
        final Method getForce  = base.getClass().getMethod("getForce");
        final Method getMoment = base.getClass().getMethod("getMoment");
        final Method getCP     = base.getClass().getMethod("getCP");
        final Coordinate F0 = (Coordinate) getForce.invoke(base);
        final Coordinate M0 = (Coordinate) getMoment.invoke(base);
        final Coordinate CP = (Coordinate) getCP.invoke(base);
        final Coordinate Ftot = (F0 != null ? F0 : Coordinate.NUL).add(dF);
        final Coordinate Mtot = (M0 != null ? M0 : Coordinate.NUL).add(dM);

        final Constructor<?> ctor = findCtor(base.getClass(),
                Coordinate.class, Coordinate.class, Coordinate.class);
        if (ctor != null && CP != null) {
            return (AerodynamicForces) ctor.newInstance(Ftot, Mtot, CP);
        }
    } catch (Throwable ignore) { /* fall through */ }

    // Otherwise, mutate in place if setters exist
    try {
        final Method setForce  = base.getClass().getMethod("setForce", Coordinate.class);
        final Method setMoment = base.getClass().getMethod("setMoment", Coordinate.class);
        final Method getForce  = base.getClass().getMethod("getForce");
        final Method getMoment = base.getClass().getMethod("getMoment");
        final Coordinate F0 = (Coordinate) getForce.invoke(base);
        final Coordinate M0 = (Coordinate) getMoment.invoke(base);
        final Coordinate Ftot = (F0 != null ? F0 : Coordinate.NUL).add(dF);
        final Coordinate Mtot = (M0 != null ? M0 : Coordinate.NUL).add(dM);
        setForce.invoke(base, Ftot);
        setMoment.invoke(base, Mtot);
        return base;
    } catch (Throwable ignore) { /* last resort below */ }

    // Last resort: bump CD/CM coefficients if present (still influences sim)
    try {
        Double cd0 = 0.0, cm0 = 0.0;
        try { cd0 = (Double) base.getClass().getMethod("getCD").invoke(base); } catch (Throwable ignored) {}
        try { cm0 = (Double) base.getClass().getMethod("getCM").invoke(base); } catch (Throwable ignored) {}
        try { if (cd0 != null) base.getClass().getMethod("setCD", double.class).invoke(base, cd0 + dCd); } catch (Throwable ignored) {}
        try { if (cm0 != null) base.getClass().getMethod("setCM", double.class).invoke(base, cm0 + dCm); } catch (Throwable ignored) {}
    } catch (Throwable ignored) {}

    return base;
}


    // ── life-cycle: end ────────────────────────
    @Override
    public void endSimulation(final SimulationStatus status, final SimulationException e) {
        if (e != null) log.error("Simulation ended with error", e);
        else           log.info("Simulation complete – final deploy={}", deploy);
    }

    // ── helpers ────────────────────────────────────────────────────────────
    private static double clamp(double x, double lo, double hi)      { return Math.min(Math.max(x, lo), hi); }
    private static double clamp01(double x)                           { return clamp(x, 0.0, 1.0); }
    private static double finiteOr(double x, double fallback)         { return Double.isFinite(x) ? x : fallback; }

    // Returns true iff OpenRocket's current apogee reading >= deploy altitude threshold
    public boolean isAirbrakeAllowedToDeploy(final SimulationStatus st) {
        if (st == null) return false;

        // get threshold from config
        double threshold = 0.0;
        try { threshold = config.getDeployAltitudeThreshold(); } catch (Throwable ignore) { }

        // get apogee from OR (try several known FlightDataType constants via reflection)
        final double apogee = getOpenRocketApogee(st);

        // Only allow if OR is providing a finite apogee and it's past the threshold
        return Double.isFinite(apogee) && apogee >= threshold;
    }

    /** Best-effort retrieval of OpenRocket's apogee value at the current step. */
    private static double getOpenRocketApogee(final SimulationStatus st) {
        if (st == null || st.getFlightDataBranch() == null) return Double.NaN;

        // Try a few likely FlightDataType constants in order of preference
        final String[] candidates = new String[] {
                "TYPE_APOGEE",            // common in some OR builds
                "TYPE_MAX_ALTITUDE",      // sometimes used instead
                "TYPE_APOGEE_ESTIMATE",   // possible predictor output
                "TYPE_APOGEE_PREDICTION"  // fallback name in some forks
        };
        for (String name : candidates) {
            Double v = tryFlightDatum(st, name);
            if (v != null) return v;
        }
        return Double.NaN;
    }

    /** Reflective getter to avoid compile-time dependency on specific FlightDataType constants. */
    private static Double tryFlightDatum(final SimulationStatus st, final String constName) {
        try {
            java.lang.reflect.Field f = info.openrocket.core.simulation.FlightDataType.class.getField(constName);
            Object ft = f.get(null);
            if (ft instanceof info.openrocket.core.simulation.FlightDataType) {
                double val = st.getFlightDataBranch().getLast((info.openrocket.core.simulation.FlightDataType) ft);
                return Double.isFinite(val) ? val : null;
            }
        } catch (Throwable ignore) {}
        return null;
    }
    
    // Helper: find a constructor by parameter types
    private static Constructor<?> findCtor(Class<?> cls, Class<?>... params) {
        try {
            return cls.getConstructor(params);
        } catch (NoSuchMethodException e) {
            for (Constructor<?> c : cls.getConstructors()) {
                Class<?>[] p = c.getParameterTypes();
                if (p.length == params.length) {
                    boolean ok = true;
                    for (int i = 0; i < p.length; i++) {
                        if (!p[i].isAssignableFrom(params[i])) { ok = false; break; }
                    }
                    if (ok) return c;
                }
            }
            return null;
        }
    }

    private static double inferSrefFromRocket(Rocket r) {
        if (r == null) return Math.PI * 0.08 * 0.08; // fallback: 160 mm dia
        try {
            double rad = r.getBoundingRadius();      // available on recent OR builds
            if (Double.isFinite(rad) && rad > 0.0) return Math.PI * rad * rad;
        } catch (Throwable ignored) {}
        return Math.PI * 0.08 * 0.08;
    }

    private static double inferLrefFromRocket(Rocket r) {
        if (r == null) return 0.16;                  // 160 mm fallback
        try {
            double L = r.getLength();
            if (Double.isFinite(L) && L > 0.0) return L;
        } catch (Throwable ignored) {}
        return 0.16;
    }
}
