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
import info.openrocket.core.util.Coordinate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Waterloo-style listener: controller+predictor loop and CDAxial override.
 * Uses FlightDataType setValue/getLast (falls back to in-memory if custom types unavailable).
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    // ---- collaborators ----
    private final AirbrakeAerodynamics airbrakes;   // provides getIncrementalCd(mach, ext) (0..1 ext)
    private final AirbrakeController   controller;  // issues extend/retract via context
    private final ApogeePredictor      predictor;   // world-Z accel (incl. g) → apogee

    // ---- gates (configurable time, fixed vz as requested) -------------------
    private final double extTimeGateS;              // apply brakes after this time (s)
    private final double vzGateMps = 18.0;          // vertical speed gate (m/s)

    // ---- One-shot LUT edge logs --------------------------------------------
    private boolean loggedMachLowOnce  = false;
    private boolean loggedMachHighOnce = false;
    private boolean loggedExtClampOnce = false;

    // ---- per-step cache ----
    private FlightConditions flightConditions = null;
    private double ext = 0.0;                       // commanded extension [0..1]
    private double lastTime = Double.NaN;
    private Double lastVz = null;

    // Fallback ref area if FlightConditions not yet available
    private final double refAreaFallback;

    // Custom columns (best-effort). If null, we skip writing/reading.
    private static final FlightDataType AIRBRAKE_EXT =
            createType("airbrakeExt", "airbrakeExt", "UNITS_RELATIVE");
    private static final FlightDataType PRED_APOGEE =
            createType("predictedApogee", "predictedApogee", "UNITS_DISTANCE");

    public AirbrakeSimulationListener(AirbrakeAerodynamics airbrakes,
                                      AirbrakeController controller,
                                      ApogeePredictor predictor,
                                      double extTimeGateSeconds,
                                      double refAreaFallback) {
        this.airbrakes = airbrakes;
        this.controller = controller;
        this.predictor = predictor;
        this.extTimeGateS = extTimeGateSeconds;
        this.refAreaFallback = refAreaFallback;
    }

    // Controller callbacks: set current extension directly.
    private final AirbrakeController.ControlContext ctx = new AirbrakeController.ControlContext() {
        @Override public void extend_airbrakes()  { ext = 1.0; }
        @Override public void retract_airbrakes() { ext = 0.0; }
        @Override public void switch_altitude_back_to_pressure() { }
    };

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        ext = 0.0;
        lastTime = status.getSimulationTime();
        lastVz = null;
        loggedMachLowOnce = loggedMachHighOnce = loggedExtClampOnce = false;

        // If controller has a context setter, install ours; else assume ctor had it.
        try {
            controller.getClass()
                      .getDeclaredMethod("setContext", AirbrakeController.ControlContext.class)
                      .invoke(controller, ctx);
        } catch (Throwable ignored) {}

        try { predictor.getClass().getMethod("reset").invoke(predictor); } catch (Throwable ignored) {}
    }

    private boolean isExtensionAllowed(SimulationStatus status) {
        final Coordinate v = status.getRocketVelocity();
        final double vz = (v != null) ? v.z : 0.0;
        return status.getSimulationTime() > extTimeGateS && vz > vzGateMps;
    }

    @Override
    public boolean preStep(SimulationStatus status) {
        // time & kinematics
        final double t  = status.getSimulationTime();
        final double dt = Double.isFinite(lastTime) ? Math.max(1e-6, t - lastTime) : 1e-3;
        lastTime = t;

        final Coordinate vel = status.getRocketVelocity();
        final Coordinate pos = status.getRocketPosition();
        final double vz = (vel != null) ? vel.z : 0.0;
        final double z  = (pos != null) ? pos.z : 0.0;

        // predictor: world-Z accel including g via dv/dt
        if (lastVz != null) {
            final double a_worldZ_incl_g = (vz - lastVz) / dt;
            predictor.update(a_worldZ_incl_g, dt, z, vz);
        }
        lastVz = vz;

        // controller (issues extend/retract via ctx)
        try { controller.updateAndGateFlexible(status); } catch (Throwable ignored) {}

        // write columns (best-effort)
        final FlightDataBranch fdb = status.getFlightDataBranch();
        if (AIRBRAKE_EXT != null) fdb.setValue(AIRBRAKE_EXT, ext);
        if (PRED_APOGEE != null) {
            Double aps = predictor.getPredictionIfReady();
            Double apb = predictor.getApogeeBestEffort();
            fdb.setValue(PRED_APOGEE, (aps != null) ? aps : (apb != null ? apb : Double.NaN));
        }

        return true;
    }

    @Override
    public FlightConditions postFlightConditions(SimulationStatus status,
                                                 FlightConditions fc) throws SimulationException {
        this.flightConditions = fc;
        return fc;
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus status,
                                                        final AerodynamicForces forces) throws SimulationException {
        if (airbrakes == null || forces == null || status == null) return forces;

        // Gate by time and vertical speed (Waterloo-style), and require some extension
        if (!isExtensionAllowed(status)) return forces;
        final double airbrakeExt = getLatestExtension(status);
        if (!(airbrakeExt > 1e-6)) return forces;

        // Velocity, altitude, Mach
        final Coordinate v = status.getRocketVelocity();
        final double v2 = (v != null) ? v.length2() : 0.0;
        final double speed = Math.sqrt(Math.max(0.0, v2));

        double altitude;
        try {
            altitude = status.getRocketPosition().z
                    + status.getSimulationConditions().getLaunchSite().getAltitude();
        } catch (Throwable t) {
            altitude = status.getRocketPosition().z;
        }

        double mach = AirDensity.machFromV(speed, altitude);
        if (!Double.isFinite(mach) || mach <= 0) {
            try {
                final double a = AirDensity.speedOfSoundISA(altitude);
                if (a > 1e-6) mach = speed / a;
            } catch (Throwable ignore) { mach = 0.0; }
        }

        // Atmospheric terms
        final double rho = (flightConditions != null)
                ? flightConditions.getAtmosphericConditions().getDensity()
                : AirDensity.rhoForDynamicPressureFromV(altitude, speed);
        final double dynP = 0.5 * rho * v2;
        final double refArea = (flightConditions != null)
                ? flightConditions.getRefArea()
                : Math.max(1e-9, refAreaFallback);
        if (!(dynP > 1e-6) || !(refArea > 0.0)) return forces;

        // Clamp Mach/extension to LUT bounds if available; log once
        Double machMinObj = tryGetDouble(airbrakes, "getMinMach", "getMachMin", "minMach", "getMachMinValue");
        Double machMaxObj = tryGetDouble(airbrakes, "getMaxMach", "getMachMax", "maxMach", "getMachMaxValue");
        double machMin = (machMinObj != null) ? machMinObj : Double.NaN;
        double machMax = (machMaxObj != null) ? machMaxObj : Double.NaN;

        double machClamped = mach;
        if (Double.isFinite(machMin) && machClamped < machMin) {
            if (!loggedMachLowOnce) {
                log.debug("[Airbrakes] LUT edge: Mach {} below min {} — clamping",
                        String.format("%.3f", machClamped), String.format("%.3f", machMin));
                loggedMachLowOnce = true;
            }
            machClamped = machMin;
        }
        if (Double.isFinite(machMax) && machClamped > machMax) {
            if (!loggedMachHighOnce) {
                log.debug("[Airbrakes] LUT edge: Mach {} above max {} — clamping",
                        String.format("%.3f", machClamped), String.format("%.3f", machMax));
                loggedMachHighOnce = true;
            }
            machClamped = machMax;
        }

        double extIn = airbrakeExt;
        double extClamped = clamp01(extIn);
        if (extClamped != extIn && !loggedExtClampOnce) {
            log.debug("[Airbrakes] LUT edge: extension {} outside [0,1] — clamping to {}",
                    String.format("%.3f", extIn), String.format("%.3f", extClamped));
            loggedExtClampOnce = true;
        }

        // ΔCd from LUT → drag force; if your class exposes calculateDragForce, you can swap it in.
        final double dCd = airbrakes.getIncrementalCd(machClamped, extClamped);
        if (!Double.isFinite(dCd) || Math.abs(dCd) < 1e-12) return forces;

        final double dragForce = dCd * dynP * refArea;
        final double cDAxial = dragForce / dynP / refArea; // equals dCd

        // *** DEBUG: verify the override applied (as requested) ***
        log.debug("[Airbrakes] CDAxial set: ext={} M={} dCd={} q={}Pa S={} -> CdAx={}",
                String.format("%.2f", extClamped),
                String.format("%.3f", machClamped),
                String.format("%.5f", dCd),
                String.format("%.1f", dynP),
                String.format("%.3f", refArea),
                String.format("%.5f", cDAxial));

        // Set CDAxial (use common setter names)
        if (!trySetDouble(forces, cDAxial, "setCDaxial", "setCDAxial", "setAxialForceCoefficient", "setDragCoefficient", "setCD")) {
            log.debug("[Airbrakes] No CDAxial/drag-coefficient setter found on AerodynamicForces; leaving base unchanged.");
        }
        return forces;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private static double clamp01(double x) { return (x < 0.0) ? 0.0 : (x > 1.0 ? 1.0 : x); }

    private static boolean trySetDouble(final Object obj, final double val, final String... setters) {
        for (String s : setters) {
            try { obj.getClass().getMethod(s, double.class).invoke(obj, val); return true; }
            catch (Throwable ignore) {}
        }
        return false;
    }

    private static Double tryGetDouble(final Object obj, final String... getters) {
        for (String g : getters) {
            try {
                Object v = obj.getClass().getMethod(g).invoke(obj);
                if (v instanceof Number) return ((Number) v).doubleValue();
            } catch (Throwable ignore) {}
        }
        return null;
    }

    private static FlightDataType createType(String key, String name, String unitConst) {
        try {
            Class<?> ug = null;
            try { ug = Class.forName("info.openrocket.core.util.UnitGroup"); } catch (ClassNotFoundException ignore) { }
            if (ug == null) try { ug = Class.forName("info.openrocket.core.units.UnitGroup"); } catch (ClassNotFoundException ignore) { }
            if (ug == null) try { ug = Class.forName("net.sf.openrocket.unit.UnitGroup"); } catch (ClassNotFoundException ignore) { }
            if (ug == null) return null;

            Object units = ug.getField(unitConst).get(null);
            for (java.lang.reflect.Method m : FlightDataType.class.getMethods()) {
                if (m.getName().equals("getType") && m.getParameterCount() == 3) {
                    Class<?>[] pt = m.getParameterTypes();
                    if (pt[0] == String.class && pt[1] == String.class && pt[2].isAssignableFrom(units.getClass())) {
                        return (FlightDataType) m.invoke(null, key, name, units);
                    }
                }
            }
        } catch (Throwable ignored) { }
        return null;
    }

    private double getLatestExtension(SimulationStatus status) {
        double e = clamp01(ext);
        if (AIRBRAKE_EXT != null) {
            try {
                double v = status.getFlightDataBranch().getLast(AIRBRAKE_EXT);
                if (Double.isFinite(v)) e = clamp01(v);
            } catch (Throwable ignore) {}
        }
        return e;
    }
}
