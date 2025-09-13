// package com.airbrakesplugin;

// import com.airbrakesplugin.util.ApogeePredictor;
// import org.junit.jupiter.api.Test;
// import org.junit.jupiter.api.BeforeEach;

// import static org.junit.jupiter.api.Assertions.*;
// import static org.mockito.Mockito.*;

// import info.openrocket.core.simulation.FlightDataBranch;
// import info.openrocket.core.simulation.FlightDataType;
// import info.openrocket.core.rocketcomponent.Rocket;
// import info.openrocket.core.simulation.SimulationStatus;
// import info.openrocket.core.util.Coordinate;

// import java.lang.reflect.Field;

// /**
//  * Integration-style unit tests for {@link AirbrakeSimulationListener#preStep}.
//  * These tests exercise:
//  *   1) Coast gating (no samples during thrust; latch after burnout).
//  *   2) Predictor usage and strict ballistic apogee accuracy.
//  *   3) Controller actions (extend/retract) with best-effort gate + hysteresis.
//  *   4) Mach gate overriding controller commands.
//  *   5) Actuator slew-rate limiter evolving physical deployment.
//  *
//  * NOTE: We avoid calling startSimulation() to skip CFD file I/O; instead we inject
//  *       aerodynamics/predictor/controller via reflection.
//  */
// public class AirbrakeSimulationListenerTest {

//     private static final double G = 9.80665;
//     private static final double DT = 0.01;

//     private AirbrakeConfig cfg;
//     private AirbrakeSimulationListener listener;
//     private SimulationStatus status;
//     private FlightDataBranch fdb;

//     @BeforeEach
//     public void setup() throws Exception {
//         cfg = new AirbrakeConfig();
//         cfg.setReferenceArea(0.008706);
//         cfg.setReferenceLength(0.1016);
//         cfg.setMaxDeploymentRate(5.0); // fast for tests unless we test slew explicitly
//         cfg.setTargetApogee(600.0);
//         cfg.setDeployAltitudeThreshold(0.01);
//         cfg.setMaxMachForDeployment(1.0);
//         cfg.setAlwaysOpenMode(false);
//         cfg.setAlwaysOpenPercentage(1.0);
//         cfg.setApogeeToleranceMeters(5.0);

//         listener = new AirbrakeSimulationListener(cfg, mock(Rocket.class));
//         status   = mock(SimulationStatus.class, withSettings().lenient());
//         fdb      = mock(FlightDataBranch.class, withSettings().lenient());

//         // minimal wiring so preStep() runs
//         inject(listener, "aerodynamics", mock(AirbrakeAerodynamics.class));
//         inject(listener, "predictor",     new ApogeePredictor());

//         // Controller uses the listener's commandedDeploy via this context
//         AirbrakeController.ControlContext ctx = new AirbrakeController.ControlContext() {
//             @Override public void extend_airbrakes()  { setField(listener, "commandedDeploy", 1.0); }
//             @Override public void retract_airbrakes() { setField(listener, "commandedDeploy", 0.0); }
//             @Override public void switch_altitude_back_to_pressure() { /* no-op */ }
//         };
//         inject(listener, "controller", new AirbrakeController(cfg.getTargetApogee(), getPredictor(), ctx, 0.5, 5.0));

//         when(status.getFlightDataBranch()).thenReturn(fdb);
//         when(fdb.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(0.30); // subsonic by default
//     }

//     // ---------- 1) NO COAST DURING THRUST; LATCH AFTER BURNOUT ----------
//     @Test
//     public void poweredFlight_hasNoCoastSamples_thenCoastLatch() {
//         // Back-solve a thrusting segment: +15 m/s^2 from t=0.95 → 1.45 s; then coast
//         final double tStart = 0.95, tBurn = 1.45, aThrust = +15.0;
//         // Choose a state that yields a plausible altitude/velocity at burnout
//         double vz = 60.0;      // m/s at tStart
//         double alt = 20.0;     // m at tStart

//         double t = tStart;
//         // Thrusting phase: stub thrust > 1N so the thrust gate stays FALSE
//         while (t < tBurn - 1e-9) {
//             step(t, alt, vz, /*thrustN*/500.0);
//             vz  += aThrust * DT;
//             alt += vz * DT;
//             t   += DT;
//         }
//         assertEquals(0, getPredictor().sampleCount(), "Predictor should have 0 samples during powered flight.");

//         // Coast phase: thrust=0, dv/dt ≈ -g → should latch and start sampling
//         for (int i = 0; i < 50; i++) {
//             step(t, alt, vz, /*thrustN*/0.0);
//             vz  += -G * DT;
//             alt += vz * DT;
//             t   += DT;
//         }
//         assertTrue(getPredictor().sampleCount() > 0, "Predictor did not start sampling after burnout.");
//     }

//     // ---------- 2) STRICT BALISTIC APOGEE ACCURACY ----------
//     @Test
//     public void ballistic_strictApogeeMatchesClosedForm() {
//         // Coast-only scenario from a high upward velocity: should be nearly perfect.
//         final double t0 = 0.0;
//         double vz = 120.0;
//         double alt = 50.0;

//         // Manually force the listener into "coast latched" to begin sampling immediately
//         setField(listener, "coastLatched", true);

//         double t = t0;
//         Double strict = null;
//         for (int i = 0; i < 4000 && strict == null; i++) {
//             step(t, alt, vz, /*thrustN*/0.0);
//             vz  += -G * DT;
//             alt += vz * DT;
//             t   += DT;
//             strict = getPredictor().getPredictionIfReady();
//         }
//         assertNotNull(strict, "Strict apogee never became available.");

//         // Ballistic closed-form apex from the CURRENT state at the time strict appeared
//         // We do not know the exact (alt,vz) at that instant; approximate using last stubbed values
//         double expected = alt + (vz * vz) / (2.0 * G);
//         assertEquals(expected, strict.doubleValue(), 5.0, "Strict apogee should match ballistic closed-form within ~5 m.");
//     }

//     // ---------- 3) CONTROLLER ACTIONS WITH HYSTERESIS (BEST-EFFORT GATE) ----------
//     @Test
//     public void controller_extends_then_retracts_withDeadband() throws Exception {
//         // Replace predictor with a scripted one that emits BEST-EFFORT only.
//         ScriptedPredictor sp = new ScriptedPredictor(
//                 new double[] { 580, 590, 605, 620, 640, 660, 670 }, // BE sequence (meters)
//                 null                                               // no STRICT here
//         );
//         inject(listener, "predictor", sp);

//         // Controller with small min window so it can act quickly
//         AirbrakeController.ControlContext ctx = new AirbrakeController.ControlContext() {
//             @Override public void extend_airbrakes()  { setField(listener, "commandedDeploy", 1.0); }
//             @Override public void retract_airbrakes() { setField(listener, "commandedDeploy", 0.0); }
//             @Override public void switch_altitude_back_to_pressure() {}
//         };
//         AirbrakeController ctrl = new AirbrakeController(600.0, sp, ctx, 0.2, 5.0);
//         ctrl.setApogeeDeadbandMeters(10.0); // ±10 m band
//         inject(listener, "controller", ctrl);

//         // Latch coast so preStep() will call predictor/controller
//         setField(listener, "coastLatched", true);

//         // Step forward: after ~0.2 s from first BE, controller may extend if BE > target+db
//         double t = 0.0, alt = 100.0, vz = 50.0;
//         for (int i = 0; i < 200; i++) {
//             step(t, alt, vz, 0.0);
//             t += DT;
//         }
//         double cmd = (double) getField(listener, "commandedDeploy");
//         assertEquals(1.0, cmd, 1e-9, "Controller should EXTEND when BE apogee > target + deadband.");

//         // Now feed a sequence that dips below target - deadband → expect RETRACT
//         sp.appendBestEffortSequence(new double[] { 590, 585, 580, 575, 570, 565, 560 });
//         for (int i = 0; i < 400; i++) {
//             step(t, alt, vz, 0.0);
//             t += DT;
//         }
//         cmd = (double) getField(listener, "commandedDeploy");
//         assertEquals(0.0, cmd, 1e-9, "Controller should RETRACT when BE apogee < target - deadband.");
//     }

//     // ---------- 4) MACH GATE BLOCKS ACTUATION ----------
//     @Test
//     public void machGate_blocksExtendEvenWhenApogeeHigh() {
//         // Raise the instantaneous Mach above limit
//         cfg.setMaxMachForDeployment(0.3);
//         when(fdb.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(0.50);

//         // Coast latched, and predictor reporting very high BE to request extension
//         setField(listener, "coastLatched", true);

//         ScriptedPredictor sp = new ScriptedPredictor(new double[] { 1000, 1100, 1200 }, null);
//         inject(listener, "predictor", sp);

//         double t = 0.0, alt = 100.0, vz = 50.0;
//         for (int i = 0; i < 200; i++) {
//             step(t, alt, vz, 0.0);
//             t += DT;
//         }
//         double cmd = (double) getField(listener, "commandedDeploy");
//         assertEquals(0.0, cmd, 1e-9, "Mach gate should hold commandedDeploy at 0 while Mach > max.");
//     }

//     // ---------- 5) ACTUATOR SLEW-RATE LIMITER ----------
//     @Test
//     public void actuator_slewsTowardCommandAtMaxRate() {
//         // Instant command to EXTEND; verify physical deployment moves by ≤ maxRate*dt
//         setField(listener, "coastLatched", true);
//         setField(listener, "commandedDeploy", 1.0);
//         cfg.setMaxDeploymentRate(0.5); // 0.5 frac/s
//         setField(listener, "maxRate", 0.5);

//         double t = 0.0, alt = 0.0, vz = 0.0;
//         step(t, alt, vz, 0.0); // first step establishes lastTime
//         t += DT;

//         double before = listener.getCurrentAirbrakeDeployment();
//         step(t, alt, vz, 0.0); // second step applies slew
//         double after  = listener.getCurrentAirbrakeDeployment();

//         double dmax = 0.5 * DT;
//         assertTrue(after - before <= dmax + 1e-9 && after > before,
//                 "Deployment should increase by at most maxRate*dt.");
//     }

//     // -------------------------- Helpers --------------------------

//     private void step(double t, double alt, double vz, double thrustN) {
//         when(status.getSimulationTime()).thenReturn(t);
//         when(status.getRocketVelocity()).thenReturn(new Coordinate(0, 0, vz));
//         when(status.getRocketPosition()).thenReturn(new Coordinate(0, 0, alt));
//         when(fdb.getLast(FlightDataType.TYPE_ALTITUDE)).thenReturn(alt);
//         when(fdb.getLast(FlightDataType.TYPE_THRUST_FORCE)).thenReturn(thrustN);

//         try {
//             listener.preStep(status);
//         } catch (Throwable ex) {
//             fail("preStep threw: " + ex);
//         }
//     }

//     private ApogeePredictor getPredictor() {
//         return (ApogeePredictor) getField(listener, "predictor");
//     }

//     private static void inject(Object target, String field, Object value) {
//         try {
//             Field f = target.getClass().getDeclaredField(field);
//             f.setAccessible(true);
//             f.set(target, value);
//         } catch (ReflectiveOperationException e) {
//             throw new AssertionError("Failed to inject " + field + ": " + e);
//         }
//     }

//     private static Object getField(Object target, String field) {
//         try {
//             Field f = target.getClass().getDeclaredField(field);
//             f.setAccessible(true);
//             return f.get(target);
//         } catch (ReflectiveOperationException e) {
//             throw new AssertionError("Failed to read " + field + ": " + e);
//         }
//     }

//     private static void setField(Object target, String field, double value) {
//         try {
//             Field f = target.getClass().getDeclaredField(field);
//             f.setAccessible(true);
//             f.setDouble(target, value);
//         } catch (ReflectiveOperationException e) {
//             throw new AssertionError("Failed to set " + field + ": " + e);
//         }
//     }

//     private static void setField(Object target, String field, boolean value) {
//         try {
//             Field f = target.getClass().getDeclaredField(field);
//             f.setAccessible(true);
//             f.setBoolean(target, value);
//         } catch (ReflectiveOperationException e) {
//             throw new AssertionError("Failed to set " + field + ": " + e);
//         }
//     }

//     // Simple scripted predictor to drive controller decisions without relying on fitting.
//     private static final class ScriptedPredictor implements ApogeePredictor {
//         private double[] bestEffortSeq;
//         private final double[] strictSeq;
//         private int k = 0;

//         ScriptedPredictor(double[] bestEffortSeq, double[] strictSeq) {
//             this.bestEffortSeq = bestEffortSeq != null ? bestEffortSeq.clone() : null;
//             this.strictSeq     = strictSeq     != null ? strictSeq.clone()     : null;
//         }

//         @Override
//         public void update(double accelInclG, double dt, double altitudeMeters, double vz) {
//             k++;
//         }

//         @Override
//         public Double getApogeeBestEffort() {
//             if (bestEffortSeq == null || bestEffortSeq.length == 0) return null;
//             int idx = Math.min(k, bestEffortSeq.length - 1);
//             return bestEffortSeq[idx];
//         }

//         @Override
//         public Double getPredictionIfReady() {
//             if (strictSeq == null || strictSeq.length == 0) return null;
//             int idx = Math.min(k, strictSeq.length - 1);
//             return strictSeq[idx];
//         }

//         void appendBestEffortSequence(double[] tail) {
//             double[] a = new double[(bestEffortSeq==null?0:bestEffortSeq.length) + tail.length];
//             int n = 0;
//             if (bestEffortSeq != null) {
//                 System.arraycopy(bestEffortSeq, 0, a, 0, bestEffortSeq.length);
//                 n = bestEffortSeq.length;
//             }
//             System.arraycopy(tail, 0, a, n, tail.length);
//             // overwrite internal array
//             this.bestEffortSeq = a;
//         }

//         // Implement other methods from ApogeePredictor as needed, or leave empty if not used in tests.
//         @Override
//         public int sampleCount() { return k; }
//     }
// }
