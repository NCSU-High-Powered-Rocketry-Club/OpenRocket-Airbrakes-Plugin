package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import info.openrocket.core.simulation.SimulationStatus;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Comprehensive, no-Mockito test suite for AirbrakeController.
 * Uses a lightweight subclass (FakeStatus) instead of dynamic proxies.
 * Each test prints a short summary to stdout to aid debugging.
 */
public class AirbrakeControllerTest {

    /** Minimal status that lets us control simulation time. */
    private static final class FakeStatus extends SimulationStatus {
        private double now = 0.0;
        public void setNow(double t) { this.now = t; }
        @Override public double getSimulationTime() { return now; }
    }

    /** Control context that records commands and timestamps. */
    private static final class Ctx implements AirbrakeController.ControlContext {
        static final class Event {
            final double t; final boolean extend;
            Event(double t, boolean extend) { this.t = t; this.extend = extend; }
            @Override public String toString() { return String.format("{t=%.3fs,%s}", t, extend ? "EXTEND" : "RETRACT"); }
        }
        final List<Event> events = new ArrayList<>();
        volatile boolean extended, retracted, switched;

        @Override public void extend_airbrakes()  { extended = true;  retracted = false; }
        @Override public void retract_airbrakes() { retracted = true; extended = false; }
        @Override public void switch_altitude_back_to_pressure() { switched = true; }

        void record(double t, boolean extend) { events.add(new Event(t, extend)); }
    }

    // ---------- Simple ballistic “coast” helper ----------

    private static final double g = 9.80665;

    private static void stepBallistic(ApogeePredictor pred, double dt, double[] alt, double[] vz) {
        final double aIncG = -g;             // up-positive accel including g
        pred.update(aIncG, dt, alt[0], vz[0]);
        vz[0]  += aIncG * dt;
        alt[0] += vz[0] * dt;
    }

    /** Small bundle to summarize a run. */
    private static final class RunResult {
        final double t, alt, vz;
        final Double apBE, apStrict;
        final boolean actedBE, actedStrict;
        final int commandCount;
        final List<Ctx.Event> events;
        RunResult(double t, double alt, double vz, Double apBE, Double apStrict,
                  boolean actedBE, boolean actedStrict, int commandCount, List<Ctx.Event> events) {
            this.t=t; this.alt=alt; this.vz=vz; this.apBE=apBE; this.apStrict=apStrict;
            this.actedBE=actedBE; this.actedStrict=actedStrict; this.commandCount=commandCount; this.events=events;
        }
        @Override public String toString() {
            return String.format("t=%.3fs alt=%.2f m vz=%.2f m/s | apBE=%s apStrict=%s | actedBE=%s actedStrict=%s | commands=%d | events=%s",
                    t, alt, vz, fmt(apBE), fmt(apStrict), actedBE, actedStrict, commandCount, events);
        }
        private static String fmt(Double x){ return x==null ? "null" : String.format("%.2f", x); }
    }

    /** Run a simple ballistic coast up to tEnd, calling the controller every step. */
    private static RunResult runCoast(AirbrakeController ctl, ApogeePredictor pred,
                                      FakeStatus status, double tEnd, double dt,
                                      double alt0, double vz0, Ctx ctx) {
        double t = 0.0;
        double[] alt = new double[]{alt0};
        double[] vz  = new double[]{vz0};

        boolean actedBestEffort = false;
        boolean actedStrict     = false;
        int commands = 0;
        boolean lastExtended = false;

        while (t <= tEnd && vz[0] > -1.0) { // run a bit past apogee
            stepBallistic(pred, dt, alt, vz);
            t += dt;
            status.setNow(t);

            boolean acted = ctl.updateAndGateFlexible(status);
            // Record events on state changes
            if (acted) {
                commands++;
                boolean nowExtended = isExtended(ctl, ctx); // based on ctx flags
                if (nowExtended != lastExtended) {
                    ctx.record(t, nowExtended);
                    lastExtended = nowExtended;
                }
                if (!pred.hasConverged()) actedBestEffort = true; else actedStrict = true;
            }
            if (actedStrict && actedBestEffort) break; // both phases observed
        }

        return new RunResult(
                t, alt[0], vz[0],
                pred.getApogeeBestEffort(),
                pred.getPredictionIfReady(),
                actedBestEffort, actedStrict, commands, ctx.events
        );
    }

    private static boolean isExtended(AirbrakeController ctl, Ctx ctx) {
        // Controller only ever leaves one of these true.
        return ctx.extended && !ctx.retracted;
    }

    // ======================== TESTS ========================

    @Test @DisplayName("Above-target: extend using best-effort, then confirm with strict")
    public void testExtendWhenAboveTarget_BEThenStrict() {
        FakeStatus status = new FakeStatus();
        Ctx ctx = new Ctx();
        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(550.0, pred, ctx, 0.5, 5.0);

        RunResult rr = runCoast(ctl, pred, status, 6.0, 0.01, 300.0, 95.0, ctx);
        System.out.println("[ExtendAboveTarget] " + rr);

        assertTrue(rr.actedBE, "Expected at least one command using BEST-EFFORT before strict convergence.");
        assertTrue(rr.actedStrict, "Expected a command again after STRICT convergence.");
        assertTrue(ctx.extended, "Airbrakes should be EXTENDED (predicted apogee > target).");
        assertTrue(rr.commandCount >= 1, "Expected at least one command.");
    }

    @Test @DisplayName("Below-target: retract and never extend")
    public void testRetractWhenBelowTarget() {
        FakeStatus status = new FakeStatus();
        Ctx ctx = new Ctx();
        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(200.0, pred, ctx, 0.5, 5.0); // target above reachable

        RunResult rr = runCoast(ctl, pred, status, 6.0, 0.01, 10.0, 15.0, ctx);
        System.out.println("[RetractBelowTarget] " + rr);

        assertTrue(ctx.retracted, "Airbrakes should be RETRACTED (predicted apogee < target).");
        assertFalse(ctx.extended, "Airbrakes must never be extended in this scenario.");
    }

    @Test @DisplayName("No samples yet: controller must hold (no action)")
    public void testNoSamples_NoAction() {
        FakeStatus status = new FakeStatus();
        Ctx ctx = new Ctx();
        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(500.0, pred, ctx, 0.5, 5.0);

        boolean acted = ctl.updateAndGateFlexible(status);
        System.out.println("[NoSamples] acted=" + acted);
        assertFalse(acted, "Controller should not act when predictor has zero samples.");
        assertFalse(ctx.extended || ctx.retracted, "No command should be issued without samples.");
    }

    @Test @DisplayName("Best-effort is gated by minCoastSeconds")
    public void testBestEffortGatedByMinCoast() {
        FakeStatus status = new FakeStatus();
        Ctx ctx = new Ctx();
        ApogeePredictor pred = new ApogeePredictor();
        double minCoast = 1.0;
        AirbrakeController ctl = new AirbrakeController(550.0, pred, ctx, minCoast, 10.0);

        double[] alt = new double[]{300.0};
        double[] vz  = new double[]{95.0};
        for (int i=0; i<50; i++) {
            stepBallistic(pred, 0.01, alt, vz);
            status.setNow((i+1)*0.01);
            boolean acted = ctl.updateAndGateFlexible(status);
            if (acted) System.out.println("[BestEffortGated] Unexpected action at t=" + status.getSimulationTime());
            assertFalse(acted, "Controller should not act on BEST-EFFORT before minCoastSeconds elapses.");
        }
        System.out.println("[BestEffortGated] Passed with no actions before minCoast=" + minCoast + " s.");
    }

    @Test @DisplayName("Command de-duplication: multiple calls, one command until sign flips")
    public void testCommandDeduplication() {
        FakeStatus status = new FakeStatus();
        Ctx ctx = new Ctx();
        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(550.0, pred, ctx, 0.2, 10.0);

        double[] alt = new double[]{300.0};
        double[] vz  = new double[]{95.0};
        int actedCount = 0;

        // Run to first command
        for (int i=0; i<300; i++) {
            stepBallistic(pred, 0.01, alt, vz);
            status.setNow((i+1)*0.01);
            boolean acted = ctl.updateAndGateFlexible(status);
            if (acted) actedCount++;
            if (actedCount > 0 && i > 20) break; // we have initial command, stop soon after
        }

        // More steps with no sign flip expected yet: should not issue repeated commands
        int actedAfter = 0;
        for (int i=0; i<200; i++) {
            stepBallistic(pred, 0.01, alt, vz);
            status.setNow(status.getSimulationTime() + 0.01);
            boolean acted = ctl.updateAndGateFlexible(status);
            if (acted) actedAfter++;
        }

        System.out.println("[Dedup] initialCommands=" + actedCount + " additionalCommands=" + actedAfter + " events=" + ctx.events);
        assertTrue(actedCount >= 1, "Expected at least one initial command.");
        assertEquals(0, actedAfter, "Expected no additional commands without a sign flip.");
    }

    @Test @DisplayName("Deadband: within +deadband of target => retract (one-sided threshold)")
    public void testDeadbandOneSided() {
        FakeStatus status = new FakeStatus();
        Ctx ctx = new Ctx();
        ApogeePredictor pred = new ApogeePredictor();

        // Build an early best-effort apogee, then set target close to it
        double[] alt = new double[]{300.0};
        double[] vz  = new double[]{70.0};
        Double apBE = null;
        for (int i=0; i<150 && apBE == null; i++) {
            stepBallistic(pred, 0.02, alt, vz);
            status.setNow((i+1)*0.02);
            apBE = pred.getApogeeBestEffort();
        }
        assertNotNull(apBE, "Expected an early best-effort estimate.");
        double target = apBE;

        AirbrakeController ctl = new AirbrakeController(target, pred, ctx, 0.0, 10.0);
        ctl.setApogeeDeadbandMeters(25.0); // extend only if ap > target + 25

        boolean acted = ctl.updateAndGateFlexible(status);
        System.out.println("[Deadband] target=" + target + " apBE=" + apBE + " acted=" + acted + " events=" + ctx.events);
        assertTrue(acted, "Controller should issue an initial command.");
        assertTrue(ctx.retracted && !ctx.extended, "Within deadband the controller retracts (one-sided threshold).");
    }

    @Test @DisplayName("Null context: state toggles still occur without NPE")
    public void testNullContext_NoNPE() {
        FakeStatus status = new FakeStatus();
        ApogeePredictor pred = new ApogeePredictor();
        AirbrakeController ctl = new AirbrakeController(550.0, pred, null, 0.2, 10.0);

        double[] alt = new double[]{300.0};
        double[] vz  = new double[]{95.0};

        int acted = 0;
        for (int i=0; i<400; i++) {
            stepBallistic(pred, 0.01, alt, vz);
            status.setNow((i+1)*0.01);
            if (ctl.updateAndGateFlexible(status)) {
                acted++;
                break;
            }
        }
        System.out.println("[NullContext] acted=" + acted);
        assertTrue(acted >= 1, "Expected at least one state change even with null context.");
    }
}
