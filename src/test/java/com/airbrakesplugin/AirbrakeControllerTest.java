package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Mockito-free tests for AirbrakeController.
 *
 * We supply a dynamic-proxy SimulationStatus that only implements getSimulationTime(),
 * since the controller only calls that method.
 */
public class AirbrakeControllerTest {

    private static final double G = 9.80665; // m/s^2 downward (use negative in aAxis)
    private final double[] timeRef = new double[]{0.0}; // single source of truth for sim time

    // --- Minimal ControlContext that records commands ---
    private static class Ctx implements AirbrakeController.ControlContext {
        boolean extended = false;
        boolean retracted = false;
        int extendCount = 0;
        int retractCount = 0;
        final List<Double> timesExtended = new ArrayList<>();
        final List<Double> timesRetracted = new ArrayList<>();
        double now = 0.0;

        void setNow(double t) { this.now = t; }

        @Override public void extend_airbrakes() {
            extended = true; retracted = false;
            extendCount++;  timesExtended.add(now);
        }
        @Override public void retract_airbrakes() {
            extended = false; retracted = true;
            retractCount++;  timesRetracted.add(now);
        }
        @Override public void switch_altitude_back_to_pressure() { /* not used */ }
    }

    // --- Simple vertical world; feed predictor then advance kinematics ---
    private static class World {
        double t, alt, vz;
        World(double t0, double alt0, double vz0) { t = t0; alt = alt0; vz = vz0; }
        void step(ApogeePredictor pred, double aAxis, double dt) {
            pred.update(aAxis, dt, alt, vz); // feed current state first
            vz  += aAxis * dt;
            alt += vz * dt;
            t   += dt;
        }
    }

    private SimulationStatus status; // dynamic proxy
    private Ctx ctx;

    @BeforeEach
    void setup() {
        ctx = new Ctx();
        status = timeOnlyStatus(timeRef);
        timeRef[0] = 0.0;
    }

    /** Create a dynamic-proxy SimulationStatus that serves only getSimulationTime(). */
    private static SimulationStatus timeOnlyStatus(double[] timeRef) {
        ClassLoader cl = SimulationStatus.class.getClassLoader();
        return (SimulationStatus) Proxy.newProxyInstance(
                cl,
                new Class[]{ SimulationStatus.class },
                (proxy, method, args) -> {
                    String name = method.getName();
                    if ("getSimulationTime".equals(name)) {
                        return timeRef[0];
                    }
                    // Return safe defaults for everything else
                    Class<?> rt = method.getReturnType();
                    if (!rt.isPrimitive()) return null;
                    if (rt == boolean.class) return false;
                    if (rt == byte.class)    return (byte)0;
                    if (rt == short.class)   return (short)0;
                    if (rt == int.class)     return 0;
                    if (rt == long.class)    return 0L;
                    if (rt == float.class)   return 0f;
                    if (rt == double.class)  return 0.0;
                    if (rt == char.class)    return '\0';
                    return null; // void
                }
        );
    }

    // ---------- Helpers ----------
    private static boolean isExtendDecision(double apogee, double target, double deadband) {
        return (apogee - target) > deadband;
    }

    private static void advance(ApogeePredictor pred, World w, double dt, double aAxis,
                                SimulationStatus status, Ctx ctx, double[] timeRef) {
        w.step(pred, aAxis, dt);
        timeRef[0] = w.t;
        ctx.setNow(w.t);
    }

    // ---------- Tests ----------

    @Test
    @DisplayName("No action before minCoast; then acts afterwards")
    void noActionBeforeMinCoast_thenActs() {
        final double target = 550.0, minCoast = 0.8, maxCoast = 5.0;

        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, minCoast, maxCoast);

        World w = new World(0.0, 300.0, 95.0);
        // Before minCoast: must not act
        for (int i = 0; i < 30; i++) { // 30*0.02 = 0.6 s
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            assertFalse(ctl.updateAndGateFlexible(status), "Should not act before minCoast");
        }
        // After minCoast: should act at least once soon
        boolean actedAfter = false;
        for (int i = 0; i < 200; i++) {
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            if (ctl.updateAndGateFlexible(status)) { actedAfter = true; break; }
        }
        assertTrue(actedAfter, "Should act after minCoast when a usable apogee is available");
        assertTrue(ctx.extendCount + ctx.retractCount >= 1);
    }

    @Test
    @DisplayName("Decision aligns with STRICT when it becomes available (no duplicate command if unchanged)")
    void strictConsistency_noDuplicate() {
        final double target = 550.0, minCoast = 0.3, maxCoast = 5.0;

        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, minCoast, maxCoast);

        World w = new World(0.0, 300.0, 95.0);

        // Run until the first action (likely best-effort) occurs after minCoast
        int issued = 0;
        for (int i = 0; i < 600 && issued == 0; i++) {
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            if (ctl.updateAndGateFlexible(status)) issued++;
        }
        assertTrue(issued > 0, "Controller should act once after minCoast.");
        final int extendsAfterFirst = ctx.extendCount;
        final int retractsAfterFirst = ctx.retractCount;

        // Continue until STRICT is available; then verify state matches STRICT decision
        boolean sawStrict = false;
        for (int i = 0; i < 1500; i++) {
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            Double strict = pred.getPredictionIfReady();
            ctl.updateAndGateFlexible(status);
            if (strict != null) {
                sawStrict = true;
                boolean shouldExtend = isExtendDecision(strict, target, 0.0);
                assertEquals(shouldExtend, ctx.extended,
                        "Once STRICT is available, commanded state should agree with STRICT decision.");
                break;
            }
        }
        assertTrue(sawStrict, "Predictor never produced a STRICT apogee.");

        // Ensure no duplicate command was sent just because STRICT appeared (if decision unchanged)
        assertTrue(ctx.extendCount == extendsAfterFirst || ctx.retractCount == retractsAfterFirst,
                "Should not issue a duplicate command if STRICT yields the same decision.");
    }

    @Test
    @DisplayName("Deadband suppresses chatter near target")
    void deadbandSuppressesChatter() {
        final double minCoast = 0.2, maxCoast = 5.0;
        ApogeePredictor pred = new ApogeePredictor();

        // Fly to get STRICT prediction
        World w = new World(0.0, 260.0, 100.0);
        Double strictAp = null;
        for (int i = 0; i < 1500 && strictAp == null; i++) {
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            strictAp = pred.getPredictionIfReady();
        }
        assertNotNull(strictAp, "Failed to obtain a STRICT apogee for deadband test.");

        // Controller with target ≈ STRICT and a deadband
        double target = strictAp;
        double deadband = 12.0; // ±12 m
        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, minCoast, maxCoast);
        ctl.setApogeeDeadbandMeters(deadband);

        // Act once to establish a state
        boolean actedOnce = false;
        for (int i = 0; i < 200 && !actedOnce; i++) {
            advance(pred, w, 0.01, -G, status, ctx, timeRef);
            actedOnce = ctl.updateAndGateFlexible(status);
        }

        int toggles = 0;
        boolean last = ctx.extended;
        // Run a few seconds; prediction should hover near target
        for (int i = 0; i < 600; i++) { // ~6 s
            advance(pred, w, 0.01, -G, status, ctx, timeRef);
            boolean acted = ctl.updateAndGateFlexible(status);
            if (acted && ctx.extended != last) { toggles++; last = ctx.extended; }
        }
        assertTrue(toggles <= 1, "Deadband should prevent frequent toggling around target.");
    }

    @Test
    @DisplayName("Under-target ⇒ retract; Over-target ⇒ extend")
    void underOverTargetDecisions() {
        final double minCoast = 0.3, maxCoast = 5.0;

        // A) target very high ⇒ retract
        {
            ApogeePredictor pred = new ApogeePredictor();
            AirbrakeController ctl = new AirbrakeController(1200.0, pred, ctx, minCoast, maxCoast);
            World w = new World(0.0, 120.0, 70.0);
            boolean acted = false;
            for (int i = 0; i < 800 && !acted; i++) {
                advance(pred, w, 0.02, -G, status, ctx, timeRef);
                acted = ctl.updateAndGateFlexible(status);
            }
            assertTrue(acted, "Should act for under-target case.");
            assertTrue(ctx.retracted, "Under target ⇒ retract.");
        }

        // Reset context
        ctx = new Ctx();

        // B) target low ⇒ extend
        {
            ApogeePredictor pred = new ApogeePredictor();
            AirbrakeController ctl = new AirbrakeController(450.0, pred, ctx, minCoast, maxCoast);
            World w = new World(0.0, 280.0, 95.0);
            boolean acted = false;
            for (int i = 0; i < 800 && !acted; i++) {
                advance(pred, w, 0.02, -G, status, ctx, timeRef);
                acted = ctl.updateAndGateFlexible(status);
            }
            assertTrue(acted, "Should act for over-target case.");
            assertTrue(ctx.extended, "Over target ⇒ extend.");
        }
    }

    @Test
    @DisplayName("Command de-duplication: no repeated commands when state unchanged")
    void deduplication() {
        final double target = 500.0, minCoast = 0.2, maxCoast = 5.0;

        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, minCoast, maxCoast);
        World w = new World(0.0, 350.0, 90.0);

        // Run until first action
        int i = 0;
        for (; i < 1500; i++) {
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            if (ctl.updateAndGateFlexible(status)) break;
        }
        int extendFirst = ctx.extendCount;
        int retractFirst = ctx.retractCount;

        // Keep running under same sign of error; no new commands expected
        for (int k = 0; k < 400; k++) {
            advance(pred, w, 0.02, -G, status, ctx, timeRef);
            ctl.updateAndGateFlexible(status);
        }
        assertEquals(extendFirst, ctx.extendCount,
                "Should not re-issue EXTEND when already extended and decision unchanged.");
        assertEquals(retractFirst, ctx.retractCount,
                "Should not re-issue RETRACT when already retracted and decision unchanged.");
    }

    @Test
    @DisplayName("Irregular solver cadence: gating is time-based, not tick-based")
    void irregularCadence_timeGated() {
        final double target = 550.0, minCoast = 0.75, maxCoast = 5.0;

        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, minCoast, maxCoast);
        World w = new World(0.0, 300.0, 95.0);

        double[] dts = { 0.01, 0.07, 0.005, 0.16, 0.02, 0.18, 0.04, 0.06, 0.03, 0.21, 0.015, 0.01 };
        boolean actedBefore = false, actedAfter = false;

        for (double dt : dts) {
            advance(pred, w, dt, -G, status, ctx, timeRef);
            boolean acted = ctl.updateAndGateFlexible(status);
            if (w.t < minCoast && acted) actedBefore = true;
            if (w.t >= minCoast && acted) actedAfter = true;
        }

        assertFalse(actedBefore, "Must not act before minCoast even with jittery dt.");
        assertTrue(actedAfter, "Should act after minCoast even with jittery dt.");
    }

    @Test
    @DisplayName("Zero samples ⇒ controller holds (no firstCoastTime, no action)")
    void zeroSamples_noAction() {
        final double target = 600.0, minCoast = 0.2, maxCoast = 5.0;

        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, minCoast, maxCoast);

        // Do not feed the predictor at all (sampleCount == 0)
        timeRef[0] = 1.5; // even if time advances, without samples controller should hold
        ctx.setNow(1.5);

        for (int i = 0; i < 10; i++) {
            assertFalse(ctl.updateAndGateFlexible(status), "With zero samples, controller should not act.");
            timeRef[0] += 0.1;
            ctx.setNow(timeRef[0]);
        }
        assertEquals(0, ctx.extendCount + ctx.retractCount);
    }
}
