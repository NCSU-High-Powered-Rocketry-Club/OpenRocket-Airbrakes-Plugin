package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import info.openrocket.core.simulation.FlightDataBranch;
import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.util.Coordinate;

import java.lang.reflect.Field;

/**
 * Drives AirbrakeSimulationListener.preStep(...) with numbers from your debug log.
 * Verifies:
 *  - No "coast" sampling before burnout (1.45 s) even though your log showed 1.005 s.
 *  - After burnout, predictor outputs apogee and controller commands extension (apogee > 548.64 m).
 */
public class AirbrakeSimulationListenerTest {

    // ---- From your Debuglog.xlsx ----
    private static final double SREF_M2        = 0.008706;
    private static final double LREF_M         = 0.101598984;
    private static final double MAX_RATE_FRACS = 4.0;
    private static final double TARGET_APOGEE  = 548.64;        // m
    private static final double T_COAST_LOG    = 1.005;         // s (bad early latch in your log)
    private static final double VZ_AT_T_COAST  = 94.93;         // m/s
    private static final double ALT_AT_T_COAST = 50.144459634329785; // m
    private static final double T_BURNOUT      = 1.45;          // s
    private static final double G              = 9.80665;       // m/s^2

    private static final double T_START        = 0.95;          // s
    private static final double A_THRUST       = +15.0;         // m/s^2 (net accel while thrusting)
    private static final double DT             = 0.01;          // s

    // Back-solved initial state to hit your logged (t=1.005, vz=94.93, alt≈50.14) under +15 m/s^2
    private double vzStart;
    private double altStart;

    @BeforeEach
    public void backSolveInitials() {
        final double dt = T_COAST_LOG - T_START;
        vzStart  = VZ_AT_T_COAST - A_THRUST * dt;
        altStart = ALT_AT_T_COAST - vzStart * dt - 0.5 * A_THRUST * dt * dt;
        assertTrue(vzStart > 0);
        assertTrue(altStart > 0);
    }

    @Test
    public void listener_blocksEarlyCoast_and_extendsAfterBurnout() throws Exception {
        // --- Configuration from log ---
        final AirbrakeConfig cfg = new AirbrakeConfig();
        cfg.setCfdDataFilePath("ignored.csv");     // we’ll inject a dummy aero object
        cfg.setReferenceArea(SREF_M2);
        cfg.setReferenceLength(LREF_M);
        cfg.setMaxDeploymentRate(MAX_RATE_FRACS);
        cfg.setTargetApogee(TARGET_APOGEE);
        cfg.setDeployAltitudeThreshold(0.01);
        cfg.setMaxMachForDeployment(1.0);
        cfg.setAlwaysOpenMode(false);
        cfg.setAlwaysOpenPercentage(1.0);
        cfg.setApogeeToleranceMeters(5.0);

        // Mock a Rocket to satisfy constructor (unused in this test path)
        final Rocket rocket = mock(Rocket.class);

        // Listener under test
        final AirbrakeSimulationListener listener = new AirbrakeSimulationListener(cfg, rocket);

        // ---- Wire internals manually (we're not calling startSimulation) ----
        // predictor
        final Field predF = AirbrakeSimulationListener.class.getDeclaredField("predictor");
        predF.setAccessible(true);
        final ApogeePredictor predictor = new ApogeePredictor();
        predF.set(listener, predictor);

        // controller with a capture context (min/max coast windows: 0.5–5.0 s for a snappier test)
        final Field ctrlF = AirbrakeSimulationListener.class.getDeclaredField("controller");
        ctrlF.setAccessible(true);
        final TestCtx ctx = new TestCtx();
        final AirbrakeController controller = new AirbrakeController(
                TARGET_APOGEE, predictor, ctx, 0.5, 5.0
        );
        ctrlF.set(listener, controller);

        // dummy aerodynamics so preStep doesn't early-return (not used by preStep)
        final Field aeroF = AirbrakeSimulationListener.class.getDeclaredField("aerodynamics");
        aeroF.setAccessible(true);
        aeroF.set(listener, mock(AirbrakeAerodynamics.class));

        // If your listener caches "sim", inject it (optional)
        final SimulationStatus status = mock(SimulationStatus.class, withSettings().lenient());
        try {
            Field simF = AirbrakeSimulationListener.class.getDeclaredField("sim");
            simF.setAccessible(true);
            simF.set(listener, status);
        } catch (NoSuchFieldException ignore) { /* ok */ }

        // Mockito FDB + Mach
        final FlightDataBranch fdb = mock(FlightDataBranch.class, withSettings().lenient());
        when(status.getFlightDataBranch()).thenReturn(fdb);
        when(fdb.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(0.30); // subsonic → controller allowed

        // ---- Phase 1: thrusting (should NOT sample predictor / latch coast) ----
        double t = T_START;
        double vz = vzStart;
        double alt = altStart;

        while (t < T_BURNOUT) {
            tick(listener, status, fdb, t, alt, vz);
            vz  += A_THRUST * DT;
            alt += vz * DT;
            t   += DT;
        }
        // Assert no predictor samples yet => no early "coast" path taken
        assertEquals(0, predictor.sampleCount(), "Predictor received coast samples before burnout — gating is wrong.");

        // ---- Phase 2: coast (−g). Now we expect predictions + a command to extend. ----
        boolean extended = false;
        for (int i = 0; i < 1200; i++) { // ~12 s
            tick(listener, status, fdb, t, alt, vz);
            vz  += -G * DT;
            alt += vz * DT;
            t   += DT;

            // As soon as best-effort or strict produces apogee above target, controller should extend
            if (ctx.extended) { extended = true; break; }
            if (vz <= 0) break; // reached apogee
        }

        assertTrue(extended, "Airbrakes were never commanded to extend even though predicted apogee > target.");
        assertTrue(ctx.extended, "Airbrakes should remain extended once apogee > target.");
    }

    private static void tick(AirbrakeSimulationListener listener,
                             SimulationStatus status,
                             FlightDataBranch fdb,
                             double t, double alt, double vz) {
        when(status.getSimulationTime()).thenReturn(t);
        when(status.getRocketVelocity()).thenReturn(new Coordinate(0, 0, vz));
        when(status.getRocketPosition()).thenReturn(new Coordinate(0, 0, alt));
        when(fdb.getLast(FlightDataType.TYPE_ALTITUDE)).thenReturn(alt);
        try {
            listener.preStep(status);
        } catch (Throwable ex) {
            fail("listener.preStep threw: " + ex);
        }
    }

    /** Capture controller commands. */
    private static final class TestCtx implements AirbrakeController.ControlContext {
        volatile boolean extended, retracted, switched;
        @Override public void extend_airbrakes() { extended = true; }
        @Override public void retract_airbrakes() { retracted = true; extended = false; }
        @Override public void switch_altitude_back_to_pressure() { switched = true; }
    }
}
