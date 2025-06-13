package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;

/**
 * Core logic for the airbrake simulation (OpenRocket 23.09).
 */
public class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    private final AirbrakeConfig config;
    private final Rocket         rocket;
    private AirbrakeAerodynamics aerodynamics;
    private AirbrakeController   controller;

    private double currentDeployment = 0.0;
    private double previousTime      = 0.0;

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    public AirbrakeSimulationListener(AirbrakeConfig config, Rocket rocket) {
        if (config == null)
            throw new IllegalArgumentException("AirbrakeConfig must not be null");

        this.config  = config;
        this.rocket  = rocket;

        // Ensure sensible reference area/length
        double diameter = (config.getReferenceLength() > 0.0) ? config.getReferenceLength() : 0.16;
        if (config.getReferenceArea()   <= 0.0)
            config.setReferenceArea(Math.PI * Math.pow(diameter / 2.0, 2));
        if (config.getReferenceLength() <= 0.0)
            config.setReferenceLength(diameter);
    }

    // ------------------------------------------------------------------------
    // Simulation life-cycle
    // ------------------------------------------------------------------------
    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        log.info("AirbrakeSimulationListener: starting with {}", config);

        try {
            aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
            log.info("Loaded CFD data from {}", config.getCfdDataFilePath());
        } catch (Exception e) {
            log.error("Unable to load CFD data – disabling airbrakes", e);
            aerodynamics = null;
        }

        controller        = new AirbrakeController(config);
        currentDeployment = 0.0;
        previousTime      = (status != null) ? status.getSimulationTime() : 0.0;
    }

    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        if (status == null || aerodynamics == null || controller == null)
            return true;

        double t  = status.getSimulationTime();
        double dt = t - previousTime;
        previousTime = t;

        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double vz   = status.getRocketVelocity().z;
        double alt  = status.getRocketPosition().z;

        double cmd  = controller.getCommandedDeployment(alt, vz, mach, currentDeployment, status);
        double maxΔ = config.getMaxDeploymentRate() * dt;

        currentDeployment += Math.max(-maxΔ, Math.min(maxΔ, cmd - currentDeployment));
        currentDeployment  = Math.max(0.0, Math.min(1.0, currentDeployment));
        return true;
    }

    // ------------------------------------------------------------------------
    // Helper math (no missing API calls!)
    // ------------------------------------------------------------------------
    private static Coordinate scale(Coordinate v, double s) {
        return new Coordinate(v.x * s, v.y * s, v.z * s);
    }
    private static Coordinate add(Coordinate a, Coordinate b) {
        return new Coordinate(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    // Optional accessors via reflection (present in some OR versions) ----------
    private static Coordinate tryGet(Object obj, String method) {
        try {
            Method m = obj.getClass().getMethod(method);
            return (Coordinate) m.invoke(obj);
        } catch (Exception ignore) {
            return null;
        }
    }

        // ------------------------------------------------------------------------
        // Main aero-patch hook (4-argument version is correct for OR 23.09)
        // ------------------------------------------------------------------------
    @Override
    public AerodynamicForces postAerodynamicCalculation(
            SimulationStatus  status,
            AerodynamicForces base) throws SimulationException {

        if (aerodynamics == null || currentDeployment < 1e-6)
            return base;

        // ─────────────────────────────────────────────────── 1 Δ-coefficients
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);

        if (Double.isNaN(dCd) || Double.isNaN(dCm))
            return base;                       // NaN guard

        // ─────────────────────────────────────────────────── 2 dimensional ΔF / ΔM
        Coordinate vVec = status.getRocketVelocity();
        double     vMag = vVec.length();
        if (vMag < 1e-3)
            return base;                       // avoid divide-by-zero

        double rho = AirDensity.getAirDensityAtAltitude(status.getRocketPosition().z);
        double q   = 0.5 * rho * vMag * vMag;

        double dDrag  = q * config.getReferenceArea() * dCd;                       // [N]
        double dPitch = q * config.getReferenceArea() * config.getReferenceLength()
                        * dCm;                                                     // [N·m]

        Coordinate dragVec   = scale(vVec, -dDrag / vMag);     // opposite v
        Coordinate momentVec = new Coordinate(dPitch, 0, 0);   // body-Y

        // ─────────────────────────────────────────────────── 3 new AerodynamicForces
        Coordinate baseF = tryGet(base, "getForce");
        Coordinate baseM = tryGet(base, "getMoment");

        try {
            if (baseF != null && baseM != null) {
                Constructor<AerodynamicForces> ctor =
                        AerodynamicForces.class.getConstructor(
                                Coordinate.class, Coordinate.class, Coordinate.class,
                                double.class,   // CD
                                double.class,   // CL
                                double.class,   // CY
                                double.class);  // Cm

                // CP accessor names differ across builds
                Coordinate cp = tryGet(base, "getCenterOfPressure");
                if (cp == null) cp = tryGet(base, "getCP");

                // Helper to fetch CL / CY if those accessors exist
                double cl = tryGetDouble(base, "getCL", 0.0);
                double cy = tryGetDouble(base, "getCY", 0.0);

                return ctor.newInstance(
                        add(baseF, dragVec),       // total force
                        add(baseM, momentVec),     // total moment
                        cp,
                        base.getCD() + dCd,        // CD
                        cl,                        // CL (leave unchanged)
                        cy,                        // CY (side-force coeff)
                        base.getCm() + dCm);       // Cm
            }
        } catch (Exception ignore) {
            // Reflection failed → fall back below
        }

        // ─────────────────────────────────────────────────── 4 coeff-only fallback
        base.setCD(base.getCD() + dCd);
        base.setCm(base.getCm() + dCm);
        return base;
    }

    /* ------------------------------------------------------------------------- */
    /* Small utility for optional double accessors                               */
    private static double tryGetDouble(Object obj, String method, double fallback) {
        try {
            Method m = obj.getClass().getMethod(method);
            return ((Number) m.invoke(obj)).doubleValue();
        } catch (Exception ignore) {
            return fallback;
        }
    }

    // ------------------------------------------------------------------------
    @Override
    public void endSimulation(SimulationStatus status, SimulationException ex) {
        if (ex != null)
            log.error("Simulation ended in error", ex);
        else
            log.info("Simulation complete; final deployment = {} %", currentDeployment * 100.0);

        aerodynamics = null;
        controller   = null;
    }
}
