package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;

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

    // ── ctor ───────────────────────────────────────────────────────────────
    public AirbrakeSimulationListener(final AirbrakeConfig cfg, final Rocket rkt) {
        this.config = Objects.requireNonNull(cfg, "AirbrakeConfig must not be null");
        this.rocket = rkt;
    }

    // ── life-cycle: begin ──────────────────────────────────────────────────
    @Override
    public void startSimulation(final SimulationStatus status) throws SimulationException {
        // 0) geometry & limits
        this.Sref = (config.getReferenceArea()   > 0.0) ? config.getReferenceArea()   : inferSrefFromRocket(rocket);
        this.Lref = (config.getReferenceLength() > 0.0) ? config.getReferenceLength() : inferLrefFromRocket(rocket);

        // Allow config in %/s or fraction/s (defensive): if >1, treat as %/s
        double rateConfigured = Math.max(0.0, config.getMaxDeploymentRate());
        this.maxRate = (rateConfigured > 1.0) ? rateConfigured / 100.0 : rateConfigured;

        // 1) subsystems
        try {
            this.aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception e) {
            log.error("Failed to load airbrake CFD data from {}: {}", config.getCfdDataFilePath(), e.toString());
            throw new SimulationException("Airbrake CFD data failed to load", e);
        }
        this.controller = new AirbrakeController(config);

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
            mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        } catch (Throwable ignore) { /* leave 0.0 */ }

        double az = Double.NaN; // prefer OR-provided accel; controller will dv/dt fallback if NaN
        try {
            final Coordinate acc = status.getRocketAcceleration();
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
    @Override
    public AerodynamicForces postAerodynamicCalculation(
            final SimulationStatus status,
            final AerodynamicForces base) throws SimulationException {

        // 0) bailouts
        if (aerodynamics == null || base == null) return base;
        if (deploy <= 1e-6) return base;

        // 1) Mach (guard early-time NaNs)
        double M = 0.0;
        try {
            double m = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
            if (Double.isFinite(m)) M = m;
        } catch (Throwable ignored) { /* leave M = 0 */ }

        // 2) ΔC lookup — table may expect 0..100 (%). Try % first, then fraction fallback.
        final double uPct = deploy * 100.0;
        double dCd = aerodynamics.getIncrementalCd(M, uPct);
        double dCm = aerodynamics.getIncrementalCm(M, uPct);

        if ((!Double.isFinite(dCd) || !Double.isFinite(dCm)) ||
            (Math.abs(dCd) + Math.abs(dCm) < 1e-10 && deploy > 1e-3)) {
            // fallback: table might actually be 0..1
            dCd = aerodynamics.getIncrementalCd(M, deploy);
            dCm = aerodynamics.getIncrementalCm(M, deploy);
        }

        if (!Double.isFinite(dCd) || !Double.isFinite(dCm)) {
            // nothing we can do safely
            return base;
        }

        // 3) ΔF, ΔM from q, Sref, Lref
        final Coordinate v = status.getRocketVelocity();
        final double V   = (v != null) ? v.length() : 0.0;
        if (!(Double.isFinite(V)) || V < 1e-3) return base;

        double rho;
        try {
            // prefer your density util with current altitude
            rho = AirDensity.getAirDensityAtAltitude(status.getRocketPosition().z);
        } catch (Throwable t) {
            rho = 1.225; // sea level fallback
        }

        final double q   = 0.5 * rho * V * V;
        final double dF_drag_mag = dCd * q * Sref;        // N
        final double dM_pitch    = dCm * q * Sref * Lref; // N·m

        // Drag direction: opposite velocity
        Coordinate vhat = (v == null || v.length() < 1e-9) ? new Coordinate(-1, 0, 0)
                                                           : v.multiply(1.0 / v.length());
        final Coordinate dF = vhat.multiply(-dF_drag_mag);
        final Coordinate dM = new Coordinate(0.0, dM_pitch, 0.0); // sign may need flip per Cm sign convention

        // 4) Try FORCE/MOMENT injection via reflection if this build supports it
        try {
            // Preferred path: read base vectors, add increments, return new AerodynamicForces(F,M,CP)
            Method getForceM  = base.getClass().getMethod("getForce");
            Method getMomentM = base.getClass().getMethod("getMoment");
            Coordinate Fbase  = (Coordinate) getForceM.invoke(base);
            Coordinate Mbase  = (Coordinate) getMomentM.invoke(base);

            Coordinate Ftot = (Fbase == null) ? dF : Fbase.add(dF);
            Coordinate Mtot = (Mbase == null) ? dM : Mbase.add(dM);

            // CP getter (handle CP / Cp)
            Coordinate cp = null;
            try { cp = (Coordinate) base.getClass().getMethod("getCP").invoke(base); }
            catch (Throwable ignored) {
                try { cp = (Coordinate) base.getClass().getMethod("getCp").invoke(base); }
                catch (Throwable ignored2) { cp = null; }
            }

            Constructor<?> ctor = findCtor(base.getClass(),
                    Coordinate.class, Coordinate.class, Coordinate.class);
            if (ctor != null && cp != null) {
                return (AerodynamicForces) ctor.newInstance(Ftot, Mtot, cp);
            }

            // Secondary: mutating setters if present
            try {
                Method setForce  = base.getClass().getMethod("setForce", Coordinate.class);
                Method setMoment = base.getClass().getMethod("setMoment", Coordinate.class);
                setForce.invoke(base, Ftot);
                setMoment.invoke(base, Mtot);
                return base;
            } catch (NoSuchMethodException ignored) {
                // fall through to coefficient augmentation
            }
        } catch (NoSuchMethodException nsme) {
            // This build likely has no vector API — fall back below
        } catch (Throwable t) {
            // Any reflection failure → fall back
        }

        // 5) Fallback: coefficient augmentation (still influences sim)
        try {
            // CD
            double cd0 = 0.0;
            try { cd0 = (double) base.getClass().getMethod("getCD").invoke(base); } catch (Throwable ignored) {}
            try { base.getClass().getMethod("setCD", double.class).invoke(base, cd0 + dCd); } catch (Throwable ignored) {}

            // CM / Cm (pitch moment coefficient)
            Double cm0 = null;
            try { cm0 = (Double) base.getClass().getMethod("getCM").invoke(base); } catch (Throwable ignored) {}
            if (cm0 == null) {
                try { cm0 = (Double) base.getClass().getMethod("getCm").invoke(base); } catch (Throwable ignored) {}
            }
            if (cm0 == null) cm0 = 0.0;

            boolean setOk = false;
            try { base.getClass().getMethod("setCM", double.class).invoke(base, cm0 + dCm); setOk = true; } catch (Throwable ignored) {}
            if (!setOk) {
                try { base.getClass().getMethod("setCm", double.class).invoke(base, cm0 + dCm); } catch (Throwable ignored) {}
            }
        } catch (Throwable ignored) { /* best effort */ }

        return base;
    }

    // ── life-cycle: end ────────────────────────────────────────────────────
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
        if (st == null || st.getFlightData() == null) return Double.NaN;

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
            java.lang.reflect.Field f = net.sf.openrocket.simulation.FlightDataType.class.getField(constName);
            Object ft = f.get(null);
            if (ft instanceof net.sf.openrocket.simulation.FlightDataType) {
                double val = st.getFlightData().getLast((net.sf.openrocket.simulation.FlightDataType) ft);
                return Double.isFinite(val) ? val : null;
            }
        } catch (Throwable ignored) { }
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
