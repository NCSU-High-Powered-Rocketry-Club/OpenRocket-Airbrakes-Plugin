package com.airbrakesplugin;

import com.airbrakesplugin.util.ApogeePredictor;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import info.openrocket.core.simulation.SimulationStatus;

/** Verifies controller uses best-effort before strict, then strict, and commands extension. */
public class AirbrakeControllerTest {

    private static final class Ctx implements AirbrakeController.ControlContext {
        volatile boolean extended, retracted, switched;
        @Override public void extend_airbrakes() { extended = true; }
        @Override public void retract_airbrakes() { retracted = true; extended = false; }
        @Override public void switch_altitude_back_to_pressure() { switched = true; }
    }

    @Test
    public void bestEffortThenStrict() {
        final double target = 550.0;
        final ApogeePredictor pred = new ApogeePredictor();
        final Ctx ctx = new Ctx();

        // Controller under test (min coasting window: 0.5–5 s)
        final AirbrakeController ctl =
                new AirbrakeController(target, pred, ctx, 0.5, 5.0);

        // Mock SimulationStatus providing time only
        final SimulationStatus status = mock(SimulationStatus.class, withSettings().lenient());
        double[] times = new double[200];
        for (int i = 0; i < times.length; i++) times[i] = i * 0.02;
        when(status.getSimulationTime()).thenAnswer(inv -> {
            int idx = timeIndex[0] < times.length ? timeIndex[0] : times.length - 1;
            return times[idx];
        });

        // Feed simple ballistic coast so apogee is far above target
        final double g = 9.80665, dt = 0.02;
        double alt = 300.0, vz = 95.0;
        boolean actedBE = false, actedStrict = false;

        for (int i = 0; i < times.length; i++) {
            timeIndex[0] = i;

            // predictor update (accel includes g)
            pred.update(-g, dt, alt, vz);
            vz  += -g * dt;
            alt += vz * dt;

            boolean acted = ctl.updateAndGateFlexible(status);
            if (acted && !pred.hasConverged()) actedBE = true;
            if (acted && pred.hasConverged())  actedStrict = true;
            if (ctx.extended && actedStrict) break;
        }

        assertTrue(actedBE,     "Controller never acted using best-effort before strict convergence.");
        assertTrue(actedStrict, "Controller never acted after strict convergence.");
        assertTrue(ctx.extended, "Airbrakes should be extended (predicted apogee > target).");
    }

    // tiny mutable index for the time Answer()
    private static final int[] timeIndex = new int[] { 0 };
}
