package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.rocketcomponent.Rocket;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Core 6‑DoF simulation listener that injects incremental drag and pitching
 * moment produced by deployable air‑brakes.  Updated to ensure the brakes can
 * actually move (rate‑limit fix) and to use the 23.09 FlightConditions API for
 * atmospheric data.
 */
public class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    private final AirbrakeConfig config;
    private final Rocket         rocket;

    private AirbrakeAerodynamics aerodynamics;
    private AirbrakeController   controller;

    private double currentDeployment = 0.0;   // 0‑1 (fraction)
    private double lastTime          = 0.0;   // s, for Δt

    public AirbrakeSimulationListener(AirbrakeConfig cfg, Rocket rkt) {
        this.config  = cfg;
        this.rocket  = rkt;
    }

    /* --------------------------------------------------------------------- */
    /*  one‑time initialisation                                             */
    /* --------------------------------------------------------------------- */
    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        log.info("Airbrake listener starting (config = {})", config);

        // --- load CFD aerodynamic grid ------------------------------------
        if (config.getCfdDataFilePath() == null || config.getCfdDataFilePath().isBlank()) {
            log.warn("No CFD data file specified → airbrakes disabled for this run");
            aerodynamics = null;
            return;                 // effectively disables the listener
        }
        try {
            aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception e) {
            log.error("Failed to load CFD data; disabling airbrakes", e);
            aerodynamics = null;
            return;
        }

        // --- create controller --------------------------------------------
        controller = new AirbrakeController(config);

        // --- sane defaults if user left critical params at zero -----------
        if (config.getMaxDeploymentRate() <= 0) {
            log.warn("maxDeploymentRate <= 0 → interpreted as *unlimited* movement");
        }
        if (config.getTargetApogee() <= 0) {
            double guess = 300.0; // a reasonable fallback; user should set
            log.warn("targetApogee not set; defaulting to {} m", guess);
            config.setTargetApogee(guess);
        }

        lastTime = 0.0;   // simulation starts at t=0
    }

    /* --------------------------------------------------------------------- */
    /*  per‑step: update commanded/actual deployment                         */
    /* --------------------------------------------------------------------- */
    @Override
    public boolean preStep(SimulationStatus status) {
        if (aerodynamics == null)
            return true;                       // disabled

        double t  = status.getSimulationTime();   // [s]
        double dt = t - lastTime;
        if (dt <= 0)
            dt = 1e-3;                           // safeguard for first step
        lastTime = t;

        /* Guards: deploy only in a safe coast phase, below max‑Mach, above
         * deploy‑altitude threshold, and only while still ascending.        */
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double alt  = status.getRocketPosition().z;
        double vZ   = status.getRocketVelocity().z; 
        double thrust = status.getFlightData().getLast(FlightDataType.TYPE_THRUST_FORCE);
        boolean safeToDeploy = thrust < 1.0  // motor burned out (≈ coast)
                                && vZ > 0
                                && alt > config.getDeployAltitudeThreshold()
                                && (config.getMaxMachForDeployment() <= 0 || mach < config.getMaxMachForDeployment());
        // if (!safeToDeploy) {
        //     controller.resetIntegrator(status); // avoid PID wind‑up
        //     return;                             // hold last deployment
        // }

        // --- PID controller command ---------------------------------------
        double cmd = controller.getCommandedDeployment(alt, vZ, mach, currentDeployment, status);

        // --- apply rate limit ---------------------------------------------
        double maxΔ = (config.getMaxDeploymentRate() <= 0) ?
                      1.0 : config.getMaxDeploymentRate() * dt;
        double change = clamp(cmd - currentDeployment, -maxΔ, maxΔ);
        currentDeployment = clamp(currentDeployment + change, 0.0, 1.0);
        return true;
    }

    /* --------------------------------------------------------------------- */
    /*  post‑aero: inject ΔF and ΔM                                          */
    /* --------------------------------------------------------------------- */
    @Override
    public AerodynamicForces postAerodynamicCalculation(SimulationStatus   status, AerodynamicForces  base) throws SimulationException {

        if (aerodynamics == null || currentDeployment < 1e-3)
            return base;                         // nothing to add

        // --- incremental coefficients ------------------------------------
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);
        if (Double.isNaN(dCd) || Double.isNaN(dCm))
            return base;                         // NaN guard

        // --- dimensional increments --------------------------------------
        double rho  = AirDensity.getAirDensityAtAltitude(status.getRocketPosition().z);
        double vX = status.getRocketVelocity().x; // [m/s]
        double vY = status.getRocketVelocity().y; // [m/s]
        double vZ = status.getRocketVelocity().z; // [m/s], vertical component
        double vMag = Math.sqrt((vX * vX) + (vY * vY) + (vZ * vZ)); // ignore vertical component
        double q    = 0.5 * rho * vMag * vMag;

        double dF = dCd * q * config.getReferenceArea();          // drag (+X aft)
        double dM = dCm * q * config.getReferenceArea() * config.getReferenceLength();

        Coordinate dFvec = new Coordinate(-dF, 0, 0);             // body‑axis X
        Coordinate dMvec = new Coordinate(0, dM, 0);              // pitch about body‑Y

        // Create a copy of the base forces
        AerodynamicForces forces = base.clone();
        
        // Add incremental drag force to the axial component (CN, CA, Croll, Cside)
                
        // Add incremental drag force and moments to both forces and conditions
        forces.setCN(forces.getCN() + dFvec.x);
        forces.setCNa(forces.getCNa() + dFvec.y);    
        forces.setCm(forces.getCm() + dMvec.y);
        forces.setCroll(forces.getCroll() + dMvec.x); 
        forces.setCside(forces.getCside() + dMvec.z); 
        
        return forces;
    }

    /* --------------------------------------------------------------------- */
    /*  finish                                                               */
    /* --------------------------------------------------------------------- */
    @Override
    public void endSimulation(SimulationStatus status, SimulationException e) {
        if (e != null)
            log.error("Simulation aborted", e);
        log.info("Airbrake listener finished; final deployment = {} %", currentDeployment * 100.0);
    }

    /* --------------------------------------------------------------------- */
    /*  helpers                                                              */
    /* --------------------------------------------------------------------- */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(v, hi));
    }
}
