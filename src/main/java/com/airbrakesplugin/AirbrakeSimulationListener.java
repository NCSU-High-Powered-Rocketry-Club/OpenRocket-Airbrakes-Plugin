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
 *  - Gates: burnout + Mach
 *  - Control: if predictedApogee(AGL) > targetApogee(AGL) => EXTEND (ext=1), else RETRACT (ext=0)
 *  - Writes airbrakeExt (always 0.0 or 1.0) and predictedApogee (AGL)
 *  - ApogeePredictor internally integrates in MSL, but we convert to AGL for control.
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // Collaborators
    private final AirbrakeAerodynamics airbrakes;
    private final AirbrakeController   controller;
    private final ApogeePredictor      predictor;
    private final AirbrakeConfig       config;
    private final double               airbrakesAreaFallback;

    // State
    private FlightConditions flightConditions = null;
    /** Current commanded extension (0 = retracted, 1 = extended). */
    private double ext = 0.0;
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
                                      double airbrakesAreaFallback,
                                      AirbrakeConfig config) {
        this.airbrakes             = airbrakes;
        this.controller            = controller;
        this.predictor             = predictor;
        this.config                = config;
        this.airbrakesAreaFallback = airbrakesAreaFallback;
    }

    // Controller callbacks: update authoritative ext
    private final AirbrakeController.ControlContext ctx = new AirbrakeController.ControlContext() {
        @Override public void extend_airbrakes()  { setExt(1.0); }
        @Override public void retract_airbrakes() { setExt(0.0); }
        @Override public void switch_altitude_back_to_pressure() {
            log.debug("Switching altitude back to pressure (no-op in OR)");
        }
    };

    private void setExt(double v) {
        final double val = v >= 0.5 ? 1.0 : 0.0;   // enforce 0/1
        if (this.ext != val) {
            log.debug("Airbrake ext setpoint change: {} -> {}", this.ext, val);
        }
        this.ext = val;
    }

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        // Initial state for each run
        setExt(0.0);
        lastTime      = status.getSimulationTime();
        lastVz        = null;
        burnoutTimeS  = null;

        // Wire controller context & reset
        if (controller != null) {
            controller.setContext(ctx);
            controller.reset();
        }
        if (predictor != null) {
            predictor.reset();
        }

        // Seed flight data to non-NaN values
        final FlightDataBranch fdb = status.getFlightDataBranch();
        if (fdb != null) {
            fdb.setValue(AIRBRAKE_EXT, 0.0);
            fdb.setValue(PRED_APOGEE, Double.NaN);
        }

        log.info("Simulation started - initial time: {}, airbrakes retracted", lastTime);
    }

    // Gating policy: burnout + Mach  (no minimum deployment altitude)
    private boolean isExtensionAllowed(final SimulationStatus status) {
        final double t = status.getSimulationTime();

        final FlightDataBranch fdb = status.getFlightDataBranch();
        if (fdb == null) {
            log.debug("No FlightDataBranch; extension not allowed");
            return false;
        }

        // Capture burnout time once (thrust <= 0)
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

        // Mach number gate (use MSL for speed of sound consistency)
        final Coordinate v = status.getRocketVelocity();
        final double speed = Math.sqrt(v.length2());
        final double altitudeMSL = status.getRocketPosition().z +
                status.getSimulationConditions().getLaunchSite().getAltitude();
        final double mach = AirDensity.machFromV(speed, altitudeMSL);

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

        final double siteAlt      = status.getSimulationConditions().getLaunchSite().getAltitude();
        final double altitudeAGL  = pos.z;
        final double altitudeMSL  = altitudeAGL + siteAlt;

        log.debug("preStep t={} dt={}  pos={}  vel={}  vz={}  altAGL={}  altMSL={}",
                t, dt, pos, vel, vz, altitudeAGL, altitudeMSL);

        final FlightDataBranch fdb = status.getFlightDataBranch();

        // Capture burnout time here as well (so override mode has it even if we skip gates)
        if (fdb != null) {
            final double thrustN_pre = fdb.getLast(FlightDataType.TYPE_THRUST_FORCE);
            if (burnoutTimeS == null && thrustN_pre <= 0.0) {
                burnoutTimeS = t;
                log.debug("Captured burnout time (preStep): {} s", burnoutTimeS);
            }
        }

        // Predictor update (world-Z acceleration including g) using finite-difference
        if (lastVz != null && predictor != null) {
            final double a_worldZ_incl_g = (vz - lastVz) / dt;
            // Uses RK4 update: (a, dtSim, z_AGL, z_MSL, vZ)
            predictor.update(a_worldZ_incl_g, dt, altitudeAGL, altitudeMSL, vz);
            log.debug("Predictor updated: az={} lastVz={} zAGL={} zMSL={}",
                    a_worldZ_incl_g, lastVz, altitudeAGL, altitudeMSL);
        }
        lastVz = vz;

        // Read predictor outputs (MSL) and convert to AGL
        Double apUsedMSL = null;
        if (predictor != null) {
            try {
                final Double apStrict = predictor.getPredictionIfReady();
                if (apStrict != null && Double.isFinite(apStrict)) {
                    apUsedMSL = apStrict;
                } else {
                    final Double apBest = predictor.getApogeeBestEffort();
                    if (apBest != null && Double.isFinite(apBest)) apUsedMSL = apBest;
                }
            } catch (Throwable ignored) {
                final Double apBest = predictor.getApogeeBestEffort();
                if (apBest != null && Double.isFinite(apBest)) apUsedMSL = apBest;
            }
        }

        Double apUsedAGL = null;
        if (apUsedMSL != null && Double.isFinite(apUsedMSL)) {
            apUsedAGL = apUsedMSL - siteAlt;  // *** convert predicted apogee from MSL → AGL ***
        }

        // Publish predicted apogee (AGL; NaN if not ready)
        if (fdb != null) {
            fdb.setValue(PRED_APOGEE, apUsedAGL != null ? apUsedAGL : Double.NaN);
        }

        if (apUsedAGL != null) {
            log.debug("Apogee(pred_AGL)={} m vs altAGL={} m  (margin_AGL={} m)",
                    apUsedAGL, altitudeAGL, apUsedAGL - altitudeAGL);
        } else {
            log.debug("Apogee(pred)=N/A (predictor not ready)");
        }

        // --- Burnout-only override (kept minimal; returns early when active) ---
        if (config != null && config.isDeployAfterBurnoutOnly()) {
            final double delay = Math.max(0.0, config.getDeployAfterBurnoutDelayS());
            double airbrake_ext_override;
            if (burnoutTimeS == null) {
                // Still burning → keep retracted
                airbrake_ext_override = 0.0;
                log.debug("Override active: waiting for burnout");
            } else {
                final double dtSinceBurnout = t - burnoutTimeS;
                if (dtSinceBurnout >= delay) {
                    airbrake_ext_override = 1.0; // force full deploy after burnout+delay
                    log.debug("Override active: burnout+delay reached (dtSinceBurnout={} >= delay={}) → EXTEND",
                            dtSinceBurnout, delay);
                } else {
                    airbrake_ext_override = 0.0; // still within delay window
                    log.debug("Override active: within delay (dtSinceBurnout={} < delay={}) → RETRACT",
                            dtSinceBurnout, delay);
                }
            }
            airbrake_ext_override = (airbrake_ext_override >= 0.5) ? 1.0 : 0.0;
            setExt(airbrake_ext_override);
            if (fdb != null) {
                fdb.setValue(AIRBRAKE_EXT, this.ext);
            }
            return true; // skip normal controller path
        }

        // Control: if apogee(AGL) > target(AGL) → EXTEND; else RETRACT (subject to gates)
        double airbrake_ext; // authoritative for this step (will be forced to 0.0/1.0)

        if (isExtensionAllowed(status)) {
            final double targetAGL = (config != null) ? config.getTargetApogee() : Double.POSITIVE_INFINITY;

            if (apUsedAGL == null || !Double.isFinite(apUsedAGL)) {
                // Predictor not ready → retract (safe default)
                airbrake_ext = 0.0;
                log.debug("Predictor not ready; retracting by default");
            } else {
                airbrake_ext = (apUsedAGL > targetAGL) ? 1.0 : 0.0;
                log.debug("Control decision: apUsedAGL={} targetAGL={} => airbrake_ext={}",
                        apUsedAGL, targetAGL, airbrake_ext);
            }
        } else {
            // Outside gates: force retract
            airbrake_ext = 0.0;
            log.debug("Outside gates; forcing retract");
        }

        // Enforce hard 0.0 / 1.0, assign to class state, and persist
        airbrake_ext = (airbrake_ext >= 0.5) ? 1.0 : 0.0;
        setExt(airbrake_ext);
        if (fdb != null) {
            fdb.setValue(AIRBRAKE_EXT, this.ext);    // also visible in flight data plots
        }

        return true;
    }

    @Override
    public FlightConditions postFlightConditions(SimulationStatus status,
                                                 FlightConditions fc) throws SimulationException {
        this.flightConditions = fc;
        log.debug("Flight conditions updated - AOA={}, refArea(rocket)={}, refArea(cfg)={}, Mach={}",
                fc.getAOA(), fc.getRefArea(),
                (config != null ? config.getReferenceArea() : Double.NaN),
                fc.getMach());
        return fc;
    }

    /**
     * Overrides the coefficient of drag after the aerodynamic calculations are done each timestep.
     * We compute combined CD from the rocket and airbrakes so OR's solver consumes it.
     */
    @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status,
                                                        final AerodynamicForces forces) throws SimulationException {
        // --- Keep original gating, but allow aero when override is "active now" ---
        boolean overrideActiveNow = false;
        if (config != null && config.isDeployAfterBurnoutOnly() && burnoutTimeS != null) {
            final double delay = Math.max(0.0, config.getDeployAfterBurnoutDelayS());
            final double dtSinceBurnout = status.getSimulationTime() - burnoutTimeS;
            overrideActiveNow = (dtSinceBurnout >= delay);
        }
        if (!(overrideActiveNow || isExtensionAllowed(status))) {
            log.debug("Extension not allowed (and override not active), no aerodynamic modifications");
            return forces;
        }

        // Latest kinematics
        final Coordinate vVec = status.getRocketVelocity();
        final double v2 = vVec.length2();
        final double speed = Math.sqrt(v2);
        final double vz = vVec.z;

        if (flightConditions == null) {
            log.debug("No FlightConditions yet; skipping aero override this step");
            return forces;
        }

        final double mach = flightConditions.getMach();

        // Latest environment and reference areas
        final double altitudeMSL = status.getRocketPosition().z +
                status.getSimulationConditions().getLaunchSite().getAltitude();

        final double rhoDyn = AirDensity.rhoForDynamicPressure(altitudeMSL, mach);
        final double dynP = 0.5 * rhoDyn * v2;

        final double rocket_area = flightConditions.getRefArea();
        double airbrakes_area = (config != null ? config.getReferenceArea() : 0.0);
        if (airbrakes_area <= 0.0) {
            airbrakes_area = airbrakesAreaFallback;
        }

        // Get latest extension from FDB
        final FlightDataBranch fdb = status.getFlightDataBranch();
        double airbrakeExt = (fdb != null ? fdb.getLast(AIRBRAKE_EXT) : this.ext);
        if (!Double.isFinite(airbrakeExt)) airbrakeExt = this.ext;
        airbrakeExt = (airbrakeExt >= 0.5) ? 1.0 : 0.0; // enforce 0/1

        // Compute drag from your ΔDrag surface
        final double dragForceN_airbrakes = airbrakes.calculateDragForce(airbrakeExt, speed, altitudeMSL);

        // Existing rocket drag (from OR) in axial and total form
        final double cd_roc_axial = forces.getCDaxial();
        final double dragForceN_roc_axial = cd_roc_axial * dynP * rocket_area;

        final double cd_roc = forces.getCD();
        final double dragForceN_roc = cd_roc * dynP * rocket_area;

        log.debug("Airbrakes Drag = {} N at ext={}, Rocket Drag = {} N (speed={} m/s, vz={} m/s, altMSL={})",
                dragForceN_airbrakes, airbrakeExt, dragForceN_roc_axial, speed, vz, altitudeMSL);

        if (dynP <= 0 || rocket_area <= 0) {
            log.debug("dynP={} or rocket_area={} non-positive; skipping aero override", dynP, rocket_area);
            return forces;
        }

        // Combine rocket + airbrake drag, then back out effective CDs on combined reference area
        final double combinedArea = rocket_area + Math.max(0.0, airbrakes_area);

        double drag_total_axial = dragForceN_roc_axial + dragForceN_airbrakes;
        double drag_total = dragForceN_roc + dragForceN_airbrakes;

        final double Cd_total_axial = drag_total_axial / (dynP * combinedArea);
        final double Cd_total = drag_total / (dynP * combinedArea);

        forces.setCDaxial(Cd_total_axial);
        forces.setCD(Cd_total);

        // Optional diagnostic: predicted apogee (AGL) this step
        if (fdb != null) {
            final double apAGL = fdb.getLast(PRED_APOGEE);
            if (Double.isFinite(apAGL)) {
                log.debug("PostAero: set CDaxial={} (was {}), set CD={} (was {}), Apogee(pred_AGL)={} m",
                        Cd_total_axial, cd_roc_axial, Cd_total, cd_roc, apAGL);
            } else {
                log.debug("PostAero: set CDaxial={} (was {}), set CD={} (was {}), Apogee(pred)=N/A",
                        Cd_total_axial, cd_roc_axial, Cd_total, cd_roc);
            }
        }

        return forces;
    }
}
