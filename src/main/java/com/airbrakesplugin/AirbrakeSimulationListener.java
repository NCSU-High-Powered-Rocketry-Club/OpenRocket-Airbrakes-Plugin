package com.airbrakesplugin;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Core logic for the airbrake simulation (OpenRocket 23.09).
 */
public class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    private final AirbrakeConfig config;
    private final Rocket         rocket;
    private AirbrakeAerodynamics  aerodynamics;
    private AirbrakeController    controller;

    private double currentDeployment = 0.0;
    private double previousTime      = 0.0;

    public AirbrakeSimulationListener(AirbrakeConfig config, Rocket rocket) {
        if (config == null) {
            throw new IllegalArgumentException("AirbrakeConfig must not be null");
        }
        this.config = config;
        this.rocket = rocket;
        double diameter = (config.getReferenceLength() > 0.0) ? config.getReferenceLength(): 0.16;
            if (config.getReferenceArea() <= 0.0) {
            config.setReferenceArea(Math.PI * Math.pow(diameter/2.0, 2));
        }
        if (config.getReferenceLength() <= 0.0) {
            config.setReferenceLength(diameter);
        }
    }

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
        controller       = new AirbrakeController(config);
        currentDeployment = 0.0;
        previousTime      = (status != null) ? status.getSimulationTime() : 0.0;
    }

    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        if (status == null || aerodynamics == null || controller == null) {
            return true;
        }
        double t  = status.getSimulationTime();
        double dt = t - previousTime;
        previousTime = t;

        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double vz = status.getRocketVelocity().z;
        double alt = status.getRocketPosition().z;

        double cmd = controller.getCommandedDeployment(alt, vz, mach, currentDeployment, status);
        double maxΔ = config.getMaxDeploymentRate() * dt;
        currentDeployment += Math.max(-maxΔ, Math.min(maxΔ, cmd - currentDeployment));
        currentDeployment  = Math.max(0.0, Math.min(1.0, currentDeployment));

        return true;
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(
            SimulationStatus   status,
            AerodynamicForces forces
    ) throws SimulationException {
        if (aerodynamics == null || currentDeployment < 1e-3) {
            return forces;
        }

        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);

        if (Double.isNaN(dCd) || Double.isNaN(dCm)) {
            log.warn("CFD returned NaN at Mach={} deploy={}", mach, currentDeployment);
            return forces;
        }

        // Increment coefficients directly
        forces.setCD( forces.getCD() + dCd );
        forces.setCm( forces.getCm() + dCm );

        return forces;
    }

    @Override
    public void endSimulation(SimulationStatus status, SimulationException ex) {
        if (ex != null) {
            log.error("Simulation ended in error", ex);
        } else {
            log.info("Simulation complete; final deployment = {}%", currentDeployment*100.0);
        }
        aerodynamics = null;
        controller   = null;
    }
}
