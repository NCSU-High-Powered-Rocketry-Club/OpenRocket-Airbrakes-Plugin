package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.CompressibleFlow;
import com.airbrakesplugin.util.HighMachExtrapolationMode;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class HighMachFlowAndConfigTest {

    @Test
    void defaultConfigAllowsMach5DeploymentAndAeroModel() {
        AirbrakeConfig cfg = new AirbrakeConfig();

        assertEquals(5.0, cfg.getMaxMachForDeployment(), 0.0);
        assertEquals(5.0, cfg.getMaxMachForAerodynamicModel(), 0.0);
        assertEquals(0.0, cfg.getMinTotalCd(), 0.0);
        assertFalse(cfg.isAllowNegativeDeltaDrag());
        assertEquals(2.0, cfg.getMaxExtrapolatedForceMultiplier(), 0.0);
        assertEquals(HighMachExtrapolationMode.BOUNDED_NONLINEAR, cfg.getHighMachExtrapolationMode());
    }

    @Test
    void localDynamicPressureIsFiniteAndPositiveThroughMach5() {
        double altitudeM = 1_000.0;
        double speedOfSound = AirDensity.speedOfSoundISA(altitudeM);

        for (double mach : new double[] {1.0, 2.0, 3.0, 5.0}) {
            double speed = mach * speedOfSound;
            double q = AirDensity.dynamicPressure(altitudeM, speed);
            double computedMach = AirDensity.machFromV(speed, altitudeM);

            assertTrue(Double.isFinite(q), "q must be finite at Mach " + mach);
            assertTrue(q > 0.0, "q must be positive at Mach " + mach);
            assertEquals(mach, computedMach, 1e-12);
        }
    }

    @Test
    void subsonicIsentropicHelpersRejectSupersonicMach() {
        assertThrows(IllegalArgumentException.class, () -> CompressibleFlow.staticPressureRatio(1.0));
        assertThrows(IllegalArgumentException.class, () -> CompressibleFlow.compressibleDynamicPressure(101_325.0, 2.0));
        assertThrows(IllegalArgumentException.class, () -> CompressibleFlow.dynamicPressureCorrection(3.0));
    }

    @Test
    void explicitSupersonicPitotHelperAcceptsSupersonicMachOnly() {
        double qc = CompressibleFlow.supersonicPitotImpactPressure(101_325.0, 2.0);

        assertTrue(Double.isFinite(qc));
        assertTrue(qc > 0.0);
        assertThrows(IllegalArgumentException.class,
                () -> CompressibleFlow.supersonicPitotImpactPressure(101_325.0, 0.8));
    }

    @Test
    void invalidDynamicPressureInputsThrow() {
        assertThrows(IllegalArgumentException.class, () -> CompressibleFlow.dynamicPressure(-1.0, 100.0));
        assertThrows(IllegalArgumentException.class, () -> CompressibleFlow.dynamicPressure(1.0, -100.0));
        assertThrows(IllegalArgumentException.class, () -> CompressibleFlow.speedOfSound(0.0));
    }
}
