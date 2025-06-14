package com.airbrakesplugin;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.rocketcomponent.Rocket;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.AirDensity;

/**
 * Air‑brake 6‑DoF listener **compatible with OpenRocket 23.09**.
 * <p>
 * Strategy:
 * <ul>
 *   <li><b>preStep</b> (boolean) maintains a rate‑limited deployment state.</li>
 *   <li><b>postAerodynamicCalculation</b> adds incremental&nbsp;C<sub>D</sub> and&nbsp;C<sub>m</sub>
 *       directly to the existing coefficient set – no custom force vectors, thus
 *       avoiding NaN blow‑ups seen in RK4.</li>
 *   <li>All API classes come from the <code>net.sf.openrocket</code> namespace.</li>
 *   <li>No use of <code>FlightConditions</code> (not present in 23.09 core).</li>
 * </ul>
 */
public class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    private final AirbrakeConfig config;

    private AirbrakeAerodynamics aerodynamics;
    private AirbrakeController   controller;

    private double currentDeployment = 0.0;   // 0…1
    private double lastTime          = 0.0;   // s

    // ─────────────────────────────────────────────────────────────────────
    public AirbrakeSimulationListener(AirbrakeConfig cfg, Rocket _unused) {
        this.config = cfg != null ? cfg : new AirbrakeConfig();
    }

    // ====================================================================
    //  startSimulation
    // ====================================================================
    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        log.info("Airbrake listener init w/ {}", config);

        try {
            aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception e) {
            log.error("CFD load failed – air‑brakes disabled", e);
            aerodynamics = null;
        }

        controller = new AirbrakeController(config);
        lastTime   = status.getSimulationTime();
    }

    // ====================================================================
    //  preStep   (must return boolean in OR 23.09)
    // ====================================================================
    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        double t  = status.getSimulationTime();
        double dt = Math.max(t - lastTime, 1 * (10 * 10 * 10));
        lastTime  = t;

        if (aerodynamics == null) return true;      // disabled – continue sim

        // Basic coast‑phase gate
        double thrust = status.getFlightData().getLast(FlightDataType.TYPE_THRUST_FORCE);
        double vZ     = status.getRocketVelocity().z;
        double alt    = status.getRocketPosition().z;
        double mach   = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);

        boolean safe = thrust < 1.0 && vZ > 0 &&
                        alt  > config.getDeployAltitudeThreshold() &&
                        (config.getMaxMachForDeployment() <= 0 || mach < config.getMaxMachForDeployment());
        // if (!safe) {
        //     controller.resetIntegrator(status);
        //     return true;
        // }

        double cmd   = controller.getCommandedDeployment(alt, vZ, mach, currentDeployment, status);
        double maxΔ  = (config.getMaxDeploymentRate() <= 0) ? 1.0 : config.getMaxDeploymentRate() * dt;
        currentDeployment = clamp(currentDeployment + clamp(cmd - currentDeployment, -maxΔ, maxΔ), 0.0, 1.0);

        return true; // proceed with step
    }

    // ====================================================================
    //  postAerodynamicCalculation  – coefficient bump
    // ====================================================================
    @Override
    public AerodynamicForces postAerodynamicCalculation(
            SimulationStatus status,
            AerodynamicForces base) {

        // Skip if listener disabled or brakes ~closed
        if (aerodynamics == null || currentDeployment < 1e-3)
            return base;

        // 1 — Incremental coefficients from CFD
        double mach = status.getFlightData()
                            .getLast(FlightDataType.TYPE_MACH_NUMBER);
        double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);
        if (Double.isNaN(dCd) || Double.isNaN(dCm))
            return base;                                 // grid hole / bad value

        // 2 — Dynamic pressure (q = ½ρV²) using your AirDensity helper
        double alt   = status.getRocketPosition().z;     // [m] AGL
        double rho   = AirDensity.getAirDensityAtAltitude(alt);
        double vMag  = status.getRocketVelocity().length(); // total speed
        double q     = 0.5 * rho * vMag * vMag;          // [Pa]

        // 3 — Convert ΔC -> dimensional ΔF, ΔM
        double dF = dCd * q * config.getReferenceArea();              // [N]
        double dM = dCm * q * config.getReferenceArea()
                        * config.getReferenceLength();             // [N·m]

        //  • Optional numeric safety caps (tweak if needed)
        dF = Math.max(-2_000, Math.min(2_000, dF));
        dM = Math.max(-20_000, Math.min(20_000, dM));

        // 4 — Body-axis vectors (OpenRocket body X points forward)
        Coordinate ΔF = new Coordinate(-dF, 0, 0);  // drag opposes +X
        Coordinate ΔM = new Coordinate( 0,  dM, 0); // pitching moment + about Y

        // 5 — Add our forces to the base forces
        // Get current forces from base object
        Coordinate baseForce = new Coordinate(base.getCN(), 0, 0);
        Coordinate baseMoment = new Coordinate(0, base.getCm(), 0);

        // Add our incremental forces
        Coordinate newForce = baseForce.add(ΔF);
        Coordinate newMoment = baseMoment.add(ΔM);
        
        base.setCN(newForce.x);
        base.setCm(newMoment.y);

        return base;
    }

    // ====================================================================
    @Override
    public void endSimulation(SimulationStatus status, SimulationException e) {
        if (e != null) log.error("Simulation aborted", e);
        log.info("Airbrake listener done; final deployment = {} %", currentDeployment * 100.0);
    }

    // ── helpers ──────────────────────────────────────────────────────────
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
