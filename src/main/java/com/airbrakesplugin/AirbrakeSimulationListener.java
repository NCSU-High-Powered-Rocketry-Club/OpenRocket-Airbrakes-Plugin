package com.airbrakesplugin;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.CompressibleFlow;

/**
 * 6-DoF air-brake listener driven by the bang-bang + predictor controller.
 */
public class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    /* ------------------------------------------------------------------ */
    private final AirbrakeConfig  config;
    private AirbrakeAerodynamics  aerodynamics;
    private AirbrakeController    controller;

    private double currentDeployment = 0.0;   // 0–1
    private double lastTime  = 0.0;           // s
    private double prevVZ    = 0.0;           // m/s

    public AirbrakeSimulationListener(AirbrakeConfig cfg, Rocket _unused) {
        this.config = (cfg != null) ? cfg : new AirbrakeConfig();
    }

    /* ================================================================== */
    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        log.info("Air-brake listener init ⇒ {}", config);

        try {
            aerodynamics = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        } catch (Exception ex) {
            log.error("CFD table load failed – air-brakes disabled", ex);
            aerodynamics = null;
        }

        controller = new AirbrakeController(config);
        lastTime   = status.getSimulationTime();
        prevVZ     = status.getRocketVelocity().z;
    }

    /* ================================================================== */
    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {

        /* Δt ----------------------------------------------------------- */
        double t  = status.getSimulationTime();
        double dt = Math.max(t - lastTime, 1e-6);
        lastTime  = t;

        if (aerodynamics == null) return true;          // plugin disabled

        /* Flight state ------------------------------------------------- */
        double vZ   = status.getRocketVelocity().z;
        double aZ   = (vZ - prevVZ) / dt;
        prevVZ      = vZ;
        double alt  = status.getRocketPosition().z;
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);

        /* Controller command ------------------------------------------ */
        double cmd = controller.getCommandedDeployment(
                alt, vZ, aZ, dt, mach, currentDeployment, status);

        /* Actuator rate limiting (fallback = 5 fr/s) ------------------ */
        double maxRate = (config.getMaxDeploymentRate() > 0)
                         ? config.getMaxDeploymentRate()
                         : 5.0;
        double maxΔ = maxRate * dt;
        currentDeployment += clamp(cmd - currentDeployment, -maxΔ, maxΔ);
        currentDeployment  = clamp(currentDeployment, 0.0, 1.0);

        /* Debug log ---------------------------------------------------- */
        if (log.isDebugEnabled()) {
            double pred = controller.getPredictedApogeeDebug();
            log.debug(String.format(
                    "t=%5.2f alt=%7.1f vZ=%6.2f aZ=%6.2f M=%4.2f cmd=%4.2f dep=%4.2f pred=%7.1f",
                    t, alt, vZ, aZ, mach, cmd, currentDeployment, pred));
        }
        return true;
    }

    /* ================================================================== */
    @Override
    public AerodynamicForces postAerodynamicCalculation(
            SimulationStatus status,
            AerodynamicForces base) {

        if (aerodynamics == null || currentDeployment < 1e-3) return base;

        /* 1 — ΔC lookup */
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);
        if (Double.isNaN(dCd) || Double.isNaN(dCm)) return base;

        /* 2 — dynamic pressure */
        double alt  = status.getRocketPosition().z;
        double vMag = status.getRocketVelocity().length();
        double rho  = AirDensity.getAirDensityAtAltitude(alt);
        double q    = 0.5 * rho * vMag * vMag;
        if (mach >= 0.3) {
            double pStatic = AirDensity.getStaticPressureAtAltitude(alt);
            q = CompressibleFlow.compressibleDynamicPressure(pStatic, mach);
        }

        /* 3 — dimensional increments */
        double dF = dCd * q * config.getReferenceArea();
        double dM = dCm * q * config.getReferenceArea() * config.getReferenceLength();
        dF = clamp(dF, -2_000,  2_000);
        dM = clamp(dM, -20_000, 20_000);

        /* 4 — body-axis vectors */
        Coordinate ΔF = new Coordinate(-dF, 0, 0);
        Coordinate ΔM = new Coordinate( 0,  dM, 0);

        /* 5 — accumulate */
        Coordinate newF = new Coordinate(base.getCN(), 0, 0).add(ΔF);
        Coordinate newM = new Coordinate(0, base.getCm(), 0).add(ΔM);
        base.setCN(newF.x);
        base.setCm(newM.y);

        return base;
    }

    /* ================================================================== */
    @Override
    public void endSimulation(SimulationStatus status, SimulationException e) {
        if (e != null) log.error("Simulation aborted", e);
        log.info("Air-brake listener done – final deployment = {:.1f} %%",
                 currentDeployment * 100.0);
    }

    /* ------------------------------------------------------------------ */
    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }
}
