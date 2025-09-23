package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.ApogeePredictor;

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
 * Predictor-driven listener:
 *  - Gates: burnout + altitude + Mach
 *  - Control: if predictedApogee > targetApogee => EXTEND (ext=1), else RETRACT (ext=0)
 *  - Writes airbrakeExt (always 0.0 or 1.0) and predictedApogee
 *  - Aerodynamics uses in-memory ext (no branch reads → no NaN)
 *  - Tracks OpenRocket altitude (MSL) as OpenRoc_alt for comparison
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
    private double ext = 0.0;                 // 0=retracted, 1=extended (authoritative)
    private double lastTime = Double.NaN;
    private Double lastVz = null;
    private Double burnoutTimeS = null;       // set when thrust first <= 0

    // Flight data columns
    private static final FlightDataType AIRBRAKE_EXT =
            FlightDataType.getType("airbrakeExt", "airbrakeExt", UnitGroup.UNITS_RELATIVE);
    private static final FlightDataType PRED_APOGEE =
            FlightDataType.getType("predictedApogee", "predictedApogee", UnitGroup.UNITS_DISTANCE);

    public AirbrakeSimulationListener(AirbrakeAerodynamics airbrakes,
                                      AirbrakeController controller,
                                      ApogeePredictor predictor,
                                      double refAreaFallback,   // retained for ctor compatibility (unused here)
                                      AirbrakeConfig config) {
        this.airbrakes  = airbrakes;
        this.controller = controller;
        this.predictor  = predictor;
        this.config     = config;
    }

    // Controller callbacks: update authoritative ext
    private final AirbrakeController.ControlContext ctx = new AirbrakeController.ControlContext() {
        @Override public void extend_airbrakes()  { setExt(1.0); }
        @Override public void retract_airbrakes() { setExt(0.0); }
        @Override public void switch_altitude_back_to_pressure() { log.debug("Switching altitude back to pressure"); }
    };

    private void setExt(double v) {
        final double val = v >= 0.5 ? 1.0 : 0.0;   // enforce 0/1
        if (this.ext != val) {
            this.ext = val;
            log.debug("Airbrakes {}", (val == 1.0) ? "EXTENDED" : "RETRACTED");
        }
    }

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        setExt(0.0);
        lastTime = status.getSimulationTime();
        lastVz = null;
        controller.setContext(ctx);
        predictor.reset();
        burnoutTimeS = null;

        // Seed flight data to non-NaN values
        final var fdb = status.getFlightDataBranch();
        if (fdb != null) {
            fdb.setValue(AIRBRAKE_EXT, 0.0);
            fdb.setValue(PRED_APOGEE, Double.NaN);
        }

        log.info("Simulation started - initial time: {}, airbrakes retracted", lastTime);
    }

    // Gating policy: burnout + altitude(AGL) + Mach
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

        // Altitude threshold from config (AGL)
        final double altitudeAGL = status.getRocketPosition().z;
        final double hMin = (config != null) ? config.getDeployAltitudeThreshold() : 0.0;
        if (!Double.isFinite(altitudeAGL) || altitudeAGL < hMin) {
            log.debug("Extension NOT allowed: altitude AGL {} < threshold {}", altitudeAGL, hMin);
            return false;
        }

        // Mach number gate (use MSL for speed of sound consistency)
        final Coordinate v = status.getRocketVelocity();
        final double speed = Math.sqrt(v.length2());
        final double OpenRoc_alt = status.getRocketPosition().z +
                status.getSimulationConditions().getLaunchSite().getAltitude();
        final double mach = AirDensity.machFromV(speed, OpenRoc_alt);

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

        final Coordinate vel = status.getRocketVelocity();
        final Coordinate pos = status.getRocketPosition();
        final double vz = vel.z;

        // --- OpenRocket altitude (MSL) for comparison to apogee (keep this label) ---
        final double OpenRoc_alt = pos.z + status.getSimulationConditions().getLaunchSite().getAltitude();

        log.debug("preStep t={} dt={}  pos={}  vel={}  vz={}  OpenRoc_alt(MSL)={}",
                t, dt, pos, vel, vz, OpenRoc_alt);

        // ---- Predictor update (world-Z acceleration including g) ----
        if (lastVz != null) {
            final double a_worldZ_incl_g = (vz - lastVz) / dt;
            predictor.update(a_worldZ_incl_g, dt, pos.z, vz);
            log.debug("Predictor updated: az={} lastVz={}", a_worldZ_incl_g, lastVz);
        }
        lastVz = vz;

        // ---- Read predictor outputs ----
        Double apUsed = null;
        try {
            // If your predictor exposes a "ready" value, try it first
            final Double apStrict = predictor.getPredictionIfReady();
            if (apStrict != null && Double.isFinite(apStrict)) {
                apUsed = apStrict;
            } else {
                final Double apBest = predictor.getApogeeBestEffort();
                if (apBest != null && Double.isFinite(apBest)) apUsed = apBest;
            }
        } catch (Throwable ignored) {
            final Double apBest = predictor.getApogeeBestEffort();
            if (apBest != null && Double.isFinite(apBest)) apUsed = apBest;
        }

        // Publish predicted apogee (NaN if not ready)
        final FlightDataBranch fdb = status.getFlightDataBranch();
        fdb.setValue(PRED_APOGEE, apUsed != null ? apUsed : Double.NaN);

        if (apUsed != null) {
            log.debug("Apogee(pred)={} m vs OpenRoc_alt={} m  (margin={} m)",
                    apUsed, OpenRoc_alt, apUsed - OpenRoc_alt);
        } else {
            log.debug("Apogee(pred)=N/A (predictor not ready)");
        }

        // --- Your control: if apogee > target → EXTEND; else RETRACT (subject to gates) ---
        double airbrake_ext; // authoritative for this step (will be forced to 0.0/1.0)

        if (isExtensionAllowed(status)) {
            final double target = (config != null) ? config.getTargetApogee() : Double.POSITIVE_INFINITY;

            if (apUsed == null || !Double.isFinite(apUsed)) {
                // Predictor not ready → retract (safe default)
                airbrake_ext = 0.0;
                log.debug("Predictor not ready; retracting by default");
            } else {
                if (apUsed > target) {
                    airbrake_ext = 1.0;
                } else {
                    airbrake_ext = 0.0;
                }
                log.debug("Control decision: apUsed={} target={} => airbrake_ext={}", apUsed, target, airbrake_ext);
            }
        } else {
            // Outside gates: force retract
            airbrake_ext = 0.0;
            log.debug("Outside gates; forcing retract");
        }

        // Enforce hard 0.0 / 1.0, assign to class state, and persist
        airbrake_ext = (airbrake_ext >= 0.5) ? 1.0 : 0.0;
        this.ext = airbrake_ext;                 // drives postAerodynamicCalculation (no NaNs)
        fdb.setValue(AIRBRAKE_EXT, this.ext);    // also visible in flight data plots

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

        // Use authoritative in-memory extension; DO NOT read the branch here
        final double airbrakeExt = this.ext;   // guaranteed 0.0 or 1.0
        if (airbrakeExt <= 0.0) {
            log.debug("Airbrake extension is 0.0, no modifications");
            return forces;
        }

        // Speed and OpenRocket altitude (MSL) for Mach
        final Coordinate v = status.getRocketVelocity();
        final double v2 = v.length2();
        final double speed = Math.sqrt(v2);
        final double OpenRoc_alt = status.getRocketPosition().z +
                status.getSimulationConditions().getLaunchSite().getAltitude();
        log.debug("Speed: {} m/s, OpenRoc_alt(MSL): {} m", speed, OpenRoc_alt);

        // Mach and atmos/area
        final double mach = AirDensity.machFromV(speed, OpenRoc_alt);
        final double rho  = flightConditions.getAtmosphericConditions().getDensity();
        final double dynP = 0.5 * rho * v2;
        final double refArea = (flightConditions != null && flightConditions.getRefArea() > 0)
                ? flightConditions.getRefArea() : 0.0;
        final double cd0 = forces.getCDaxial();

        log.debug("Atmospheric conditions - density: {}, dynamic pressure: {}, refArea: {}", rho, dynP, refArea);

        // ΔCd from LUT
        final double dCd = airbrakes.getIncrementalCd(mach, airbrakeExt);
        log.debug("Incremental Cd from airbrakes: {} (mach: {}, ext: {})", dCd, mach, airbrakeExt);

        // Apply ΔCd to CDAxial
        final double cDAxial = cd0 + dCd;
        log.debug("Setting new CDaxial: {} (old value: {})", cDAxial, forces.getCDaxial());
        forces.setCDaxial(cDAxial);

        // Optional diagnostic: predicted apogee this step
        final double ap = status.getFlightDataBranch().getLast(PRED_APOGEE);
        if (Double.isFinite(ap)) {
            log.debug("PostAero: Apogee(pred)={} m", ap);
        }

        return forces;
    }
}

