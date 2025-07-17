package com.airbrakesplugin;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.CompressibleFlow;

/**
 * 6-DoF air-brake listener driven by the bang-bang + predictor controller.
 */
public class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    private final AirbrakeConfig    config;
    private AirbrakeAerodynamics      aerodynamics;
    private AirbrakeController        controller;

    private double currentDeployment = 0.0;   // 0–1
    private double lastTime          = 0.0;   // s
    private double prevVZ            = 0.0;   // m/s
    private double targetDeployment  = 0.0;   // 0–1 (for rate limiting)

    public AirbrakeSimulationListener(AirbrakeConfig cfg, Rocket _unused) {
        this.config = (cfg != null) ? cfg : new AirbrakeConfig();
    }

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

    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        // Time step
        double t  = status.getSimulationTime();
        double dt = Math.max(t - lastTime, 1e-6);
        lastTime  = t;

        if (aerodynamics == null) {
            return true; // plugin disabled
        }

        // Flight state
        double vZ   = status.getRocketVelocity().z;
        double aZ   = (vZ - prevVZ) / dt;
        prevVZ      = vZ;
        double alt  = status.getRocketPosition().z;
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);

        // Controller command
        double cmd = controller.getCommandedDeployment(
                alt, vZ, aZ, dt, mach, currentDeployment, status);

        // Safety validation of controller output
        if (Double.isNaN(cmd) || Double.isInfinite(cmd)) {
            log.warn("Controller returned invalid command: {}, maintaining current deployment", cmd);
            cmd = currentDeployment;
        }
        
        // Clamp command to valid range
        cmd = clamp(cmd, 0.0, 1.0);

        // Bang-bang actuator with rate limiting: fully deploy or retract each time step
        targetDeployment = (cmd >= 0.5) ? 1.0 : 0.0;
        
        // Apply rate limiting to prevent sudden changes
        double maxRate = config.getMaxDeploymentRate(); // fraction per second
        double maxChange = maxRate * dt;
        
        if (Math.abs(targetDeployment - currentDeployment) > maxChange) {
            if (targetDeployment > currentDeployment) {
                currentDeployment += maxChange;
            } else {
                currentDeployment -= maxChange;
            }
        } else {
            currentDeployment = targetDeployment;
        }
        
        // Ensure deployment stays in bounds
        currentDeployment = clamp(currentDeployment, 0.0, 1.0);

        // Additional safety check - don't deploy if conditions are extreme
        if (Math.abs(vZ) > 2000 || Math.abs(aZ) > 50000 || mach > 5.0) {
            log.warn("Extreme flight conditions detected (vZ={}, aZ={}, M={}), forcing retracted position", 
                     vZ, aZ, mach);
            currentDeployment = 0.0;
        }

        // Debug log
        if (log.isDebugEnabled()) {
            double pred = controller.getPredictedApogeeDebug();
            log.debug(String.format(
                    "t=%5.2f alt=%7.1f vZ=%6.2f aZ=%6.2f M=%4.2f cmd=%4.2f dep=%4.2f pred=%7.1f",
                    t, alt, vZ, aZ, mach, cmd, currentDeployment, pred));
        }
        return true;
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(
            SimulationStatus status,
            AerodynamicForces base) {

        if (aerodynamics == null || currentDeployment < 1e-3) {
            return base;
        }

        // 1 — ΔC lookup
        double mach = status.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);
        
        // DEBUG: Log the coefficient increments
        log.debug("AIRBRAKE CFD lookup: mach={}, deploy={}, dCd={}, dCm={}", 
                  mach, currentDeployment, dCd, dCm);
        
        if (Double.isNaN(dCd) || Double.isNaN(dCm)) {
            return base;
        }

        // 2 — dynamic pressure with safety limits
        double alt  = status.getRocketPosition().z;
        double vMag = status.getRocketVelocity().length();
        double rho  = AirDensity.getAirDensityAtAltitude(alt);
        
        // Safety limits on velocity magnitude to prevent extreme dynamic pressure
        vMag = clamp(vMag, 0.0, 1500.0);  // Cap at ~Mach 4.5 at sea level
        
        double q    = 0.5 * rho * vMag * vMag;
        if (mach >= 0.3) {
            double pStatic = AirDensity.getStaticPressureAtAltitude(alt);
            q = CompressibleFlow.compressibleDynamicPressure(pStatic, mach);
        }
        
        // DEBUG: Log dynamic pressure calculation
        log.debug("AIRBRAKE q calculation: alt={}, vMag={}, rho={}, q={}", 
                  alt, vMag, rho, q);
        
        // Additional safety check on dynamic pressure
        
        if (Double.isNaN(q) || Double.isInfinite(q) || q <= 0) {
            log.warn("Invalid dynamic pressure: {}, returning base forces", q);
            return base;
        }

        // 3 — dimensional increments
        double dF = dCd * q * config.getReferenceArea();
        double dM = dCm * q * config.getReferenceArea() * config.getReferenceLength();
        
        // DEBUG: Log the actual forces
        log.debug("AIRBRAKE forces: refArea={}, refLen={}, dF={}, dM={}", 
                  config.getReferenceArea(), config.getReferenceLength(), dF, dM);
        
        
        // Validate forces are reasonable
        if (Double.isNaN(dF) || Double.isNaN(dM) || Double.isInfinite(dF) || Double.isInfinite(dM)) {
            log.warn("Invalid airbrake forces: dF={}, dM={}, returning base forces", dF, dM);
            return base;
        }

        // 4 — convert to coefficient increments (normalize by dynamic pressure and reference area)
        double dCD = dF / (q * config.getReferenceArea());  // Renamed from dCN to dCD for clarity
        double dCm_normalized = dM / (q * config.getReferenceArea() * config.getReferenceLength());
        
        // DEBUG: Log the normalized coefficients
        log.debug("AIRBRAKE coefficients: dCD={}, dCm_normalized={}", dCD, dCm_normalized);
        
        // Additional safety check on coefficients
        dCD = clamp(dCD, -2.0, 2.0);  // Reasonable coefficient limits

        // 5 — accumulate with existing coefficients
        log.debug("AIRBRAKE before: CD={}, Cm={}", base.getCD(), base.getCm());
        base.setCD(base.getCD() + dCD);  // This was the issue in your earlier code
        base.setCm(base.getCm() + dCm_normalized);
        log.debug("AIRBRAKE after: CD={}, Cm={}", base.getCD(), base.getCm());

        return base;
    }

    @Override
    public void endSimulation(SimulationStatus status, SimulationException e) {
        if (e != null) {
            log.error("Simulation aborted", e);
        }
        log.info("Air-brake listener done – final deployment = {}%",
                 (int)(currentDeployment * 100));
    }

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }
}
