package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.ApogeePredictor;
import com.airbrakesplugin.AirbrakeConfig;

import info.openrocket.core.aerodynamics.AerodynamicForces;
import info.openrocket.core.aerodynamics.FlightConditions;
import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.unit.UnitGroup;
import info.openrocket.core.util.Coordinate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Waterloo-style listener:
 *  - Gate controller by burnout + altitude + Mach (no velocity gate)
 *  - Write airbrakeExt and predictedApogee to flight data
 *  - Override CDAxial using ΔCd(mach, ext) after aero calc
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // Collaborators
    private final AirbrakeAerodynamics airbrakes;
    private final AirbrakeController   controller;
    private final ApogeePredictor      predictor;
    private final AirbrakeConfig       config;

    // State
    private FlightConditions flightConditions = null;
    private double ext = 0.0;
    private double lastTime = Double.NaN;
    private Double lastVz = null;
    private Double burnoutTimeS = null; // set when thrust first <= 0

    // Flight data columns
    private static final FlightDataType AIRBRAKE_EXT =
            FlightDataType.getType("airbrakeExt", "airbrakeExt", UnitGroup.UNITS_RELATIVE);
    private static final FlightDataType PRED_APOGEE =
            FlightDataType.getType("predictedApogee", "predictedApogee", UnitGroup.UNITS_DISTANCE);

    public AirbrakeSimulationListener(AirbrakeAerodynamics airbrakes,  AirbrakeController controller,  ApogeePredictor predictor,  double refAreaFallback,  AirbrakeConfig config) {
        this.airbrakes = airbrakes;
        this.controller = controller;
        this.predictor = predictor;
        this.config = config;
    }

    // Controller callbacks: set current extension directly (bang-bang 0/1)
    private final AirbrakeController.ControlContext ctx = new AirbrakeController.ControlContext() {
        @Override public void extend_airbrakes()  {
            ext = 1.0;
            log.debug("Airbrakes EXTENDED");
        }
        @Override public void retract_airbrakes() {
            ext = 0.0;
            log.debug("Airbrakes RETRACTED");
        }
        @Override public void switch_altitude_back_to_pressure() {
            log.debug("Switching altitude back to pressure");
        }
    };

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        ext = 0.0;
        lastTime = status.getSimulationTime();
        lastVz = null;
        controller.setContext(ctx);
        predictor.reset();
        burnoutTimeS = null;
        log.info("Simulation started - initial time: {}, airbrakes retracted", lastTime);
    }

    // Gating policy:
    //  - NO velocity gate
    //  - Require actual MOTOR BURNOUT (anchored to instant thrust ≤ 0)
    //  - Require altitude ≥ config.getDeployAltitudeThreshold()
    //  - Require Mach ≤ config.getMaxMachForDeployment()
    private boolean isExtensionAllowed(final SimulationStatus status) {
        final double t = status.getSimulationTime();

        // Capture burnout time once
        final FlightDataBranch fdb = status.getFlightDataBranch();
        final double thrustN = fdb.getLast(FlightDataType.TYPE_THRUST_FORCE);
        if (burnoutTimeS == null && thrustN <= 0.0) {
            burnoutTimeS = t;
            log.debug("Captured burnout time: {} s", burnoutTimeS);
        }

        // Must have observed burnout, and we must be at/after it
        if (burnoutTimeS == null || t < burnoutTimeS) {
            log.debug("Extension NOT allowed: waiting for burnout (t={}, burnoutTimeS={})", t, burnoutTimeS);
            return false;
        }

        // Altitude threshold from config (meters AGL based on pos.z usage)
        final double altitude = status.getRocketPosition().z;
        final double hMin = (config != null) ? config.getDeployAltitudeThreshold() : 0.0;
        if (!Double.isFinite(altitude) || altitude < hMin) {
            log.debug("Extension NOT allowed: altitude {} < threshold {}", altitude, hMin);
            return false;
        }

        // Mach number (prefer FlightConditions if we have it)
        double mach;
        if (this.flightConditions != null) {
            mach = this.flightConditions.getMach();
        } else {
            final Coordinate v = status.getRocketVelocity();
            final double speed = Math.sqrt(v.length2());
            mach = AirDensity.machFromV(speed, altitude);
        }
        final double machMax = (config != null) ? config.getMaxMachForDeployment() : Double.POSITIVE_INFINITY;
        if (!Double.isFinite(mach) || mach > machMax) {
            log.debug("Extension NOT allowed: mach {} > max {}", mach, machMax);
            return false;
        }

        return true;
    }

    @Override
    public boolean preStep(SimulationStatus status) {
        final double t  = status.getSimulationTime();
        final double dt = Double.isFinite(lastTime) ? Math.max(1e-6, t - lastTime) : 1e-3;
        lastTime = t;

        log.debug("preStep - time: {}, dt: {}", t, dt);

        final Coordinate vel = status.getRocketVelocity();
        final Coordinate pos = status.getRocketPosition();
        final double vz = vel.z;
        final double z  = pos.z;

        log.debug("Rocket state - position: {}, velocity: {}, vz: {}, altitude: {}",
                  pos, vel, vz, z);

        if (lastVz != null) {
            final double a_worldZ_incl_g = (vz - lastVz) / dt;
            predictor.update(a_worldZ_incl_g, dt, z, vz);
            log.debug("Predictor updated - acceleration_z: {}, lastVz: {}", a_worldZ_incl_g, lastVz);
        }
        lastVz = vz;

        final FlightDataBranch fdb = status.getFlightDataBranch();
        if (isExtensionAllowed(status)) {
        // Let the controller decide; it will drive ctx.extend_/retract_ → which sets `ext`
            controller.updateAndGateFlexible(status);
            ext = 1.0;
        } else {
            // Force retract through the same callback path the controller uses
            ctx.retract_airbrakes();
            ext = 0.0;
            log.debug("Extension not allowed, forcing retraction");
        }

        // Persist current extension (for post-aero)
        fdb.setValue(AIRBRAKE_EXT, ext);

        // Graph predictor output (simple, no readiness checks)
        Double ap = predictor.getApogeeBestEffort();
        log.debug("Predicted apogee: {}", ap != null ? ap : "N/A");
        fdb.setValue(PRED_APOGEE, ap != null ? ap : Double.NaN);

        return true;
    }

    @Override
    public FlightConditions postFlightConditions(SimulationStatus status,
                                                 FlightConditions fc) throws SimulationException {
        this.flightConditions = fc;
        log.debug("Flight conditions updated - AOA: {}, refArea: {}, mach: {}",
                  fc.getAOA(), fc.getRefArea(), fc.getMach());
        return fc;
    }

     @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status,
                                                        final AerodynamicForces forces) throws SimulationException {
        log.debug("Post-aerodynamic calculation - initial CDaxial: {}, CN: {}", 
                  forces.getCDaxial(), forces.getCN());
        
        if (!isExtensionAllowed(status)) {
            log.debug("Extension not allowed, no aerodynamic modifications");
            return forces;
        }

        // Latest extension (from this step)
        final double airbrakeExt = status.getFlightDataBranch().getLast(AIRBRAKE_EXT);
        log.debug("Current airbrake extension: {}", airbrakeExt);
        
        if (airbrakeExt <= 1e-6) {
            log.debug("Airbrake extension negligible, no modifications");
            return forces;
        }

        // Speed and altitude
        final Coordinate v = status.getRocketVelocity();
        final double v2 = v.length2();
        final double speed = Math.sqrt(v2);

        final double altitude = status.getRocketPosition().z
                + status.getSimulationConditions().getLaunchSite().getAltitude();
        
        log.debug("Speed: {} m/s, altitude: {} m", speed, altitude);

        // Mach
        final double mach = AirDensity.machFromV(speed, altitude);
        log.debug("Calculated Mach number: {}", mach);

        // Atmospheric terms
        final double rho = flightConditions.getAtmosphericConditions().getDensity();
        final double dynP = 0.5 * rho * v2;
        final double refArea = flightConditions.getRefArea();
        double cd0 = forces.getCDaxial();
        log.debug("Atmospheric conditions - density: {}, dynamic pressure: {}, refArea: {}", 
                  rho, dynP, refArea);

        // ΔCd from LUT
        final double dCd = airbrakes.getIncrementalCd(mach, airbrakeExt);
        log.debug("Incremental Cd from airbrakes: {} (mach: {}, ext: {})", dCd, mach, airbrakeExt);
        
        // final double d_drag_force = 1.28 * dCd * dynP * refArea;
        // log.debug("Additional drag force: {}", d_drag_force);
        
        // Interpret ΔCd as CDAxial override (consistent with prior behavior)
        final double cDAxial = cd0 + dCd;
        log.debug("Setting new CDaxial: {} (old value: {})", cDAxial, forces.getCDaxial());

        // Direct setter (no reflection variants)
        forces.setCDaxial(cDAxial);
        return forces;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    // private static double clamp01(double x) {
    //     double result = (x < 0.0) ? 0.0 : (x > 1.0 ? 1.0 : x);
    //     if (result != x) {
    //         log.debug("Clamped value {} to {}", x, result);
    //     }
    //     return result;
    // }
}
