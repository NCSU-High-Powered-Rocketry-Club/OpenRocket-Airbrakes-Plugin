package com.airbrakesplugin;

import com.airbrakesplugin.util.GenericFunction2D;
import com.airbrakesplugin.util.HighMachExtrapolationMode;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

class AirbrakeAerodynamicsHighMachTest {

    @TempDir
    Path tempDir;

    @Test
    void positiveFixtureReturnsExactSupersonicAndMach5Forces() throws Exception {
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(configFor(resource("cfd-positive-mach0-5.csv")));

        assertEquals(90.0, aero.getAirbrakeDeltaDragN(2.0, 1.0), 1e-6);
        assertEquals(30.0, aero.getAirbrakeDeltaDragN(5.0, 1.0), 1e-6);
        assertEquals(30.0, aero.getAirbrakeDeltaDragN(3.0, 0.5), 1e-6);
        assertEquals(GenericFunction2D.EvaluationMode.INTERPOLATED, aero.getLastEvaluation().mode());
    }

    @Test
    void deploymentClampsToOne() throws Exception {
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(configFor(resource("cfd-positive-mach0-5.csv")));

        assertEquals(90.0, aero.getAirbrakeDeltaDragN(2.0, 1.25), 1e-6);
    }

    @Test
    void nonFiniteMachFailsClosed() throws Exception {
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(configFor(resource("cfd-positive-mach0-5.csv")));

        assertEquals(0.0, aero.getAirbrakeDeltaDragN(Double.NaN, 1.0), 0.0);
        assertEquals(GenericFunction2D.EvaluationMode.FAIL_CLOSED, aero.getLastEvaluation().mode());
    }

    @Test
    void negativeDeltaDragIsRejectedByDefault() throws Exception {
        AirbrakeConfig cfg = configFor(resource("cfd-negative-mach0-5.csv"));

        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> new AirbrakeAerodynamics(cfg));
        assertTrue(ex.getMessage().contains("negative delta-drag"));
    }

    @Test
    void negativeDeltaDragCanBeEnabledExplicitly() throws Exception {
        AirbrakeConfig cfg = configFor(resource("cfd-negative-mach0-5.csv"));
        cfg.setAllowNegativeDeltaDrag(true);
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(cfg);

        assertEquals(-10.0, aero.getAirbrakeDeltaDragN(2.0, 1.0), 1e-6);
        assertEquals(0.0, aero.getAirbrakeDragN(2.0, 1.0), 1e-6);
    }

    @Test
    void failClosedModeReturnsZeroOutsideTable() throws Exception {
        AirbrakeConfig cfg = configFor(resource("cfd-positive-mach0-5.csv"));
        cfg.setHighMachExtrapolationMode(HighMachExtrapolationMode.FAIL_CLOSED);
        cfg.setMaxMachForAerodynamicModel(6.0);
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(cfg);

        assertEquals(0.0, aero.getAirbrakeDeltaDragN(5.5, 1.0), 0.0);
        assertEquals(GenericFunction2D.EvaluationMode.FAIL_CLOSED, aero.getLastEvaluation().mode());
    }

    @Test
    void constantModeClampsOutsideTable() throws Exception {
        AirbrakeConfig cfg = configFor(resource("cfd-positive-mach0-5.csv"));
        cfg.setHighMachExtrapolationMode(HighMachExtrapolationMode.CONSTANT);
        cfg.setMaxMachForAerodynamicModel(6.0);
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(cfg);

        assertEquals(30.0, aero.getAirbrakeDeltaDragN(5.5, 1.0), 1e-6);
        assertEquals(GenericFunction2D.EvaluationMode.CLAMPED, aero.getLastEvaluation().mode());
    }

    @Test
    void boundedNonlinearModeExtrapolatesFiniteWithinModelLimit() throws Exception {
        AirbrakeConfig cfg = configFor(resource("cfd-positive-mach0-5.csv"));
        cfg.setHighMachExtrapolationMode(HighMachExtrapolationMode.BOUNDED_NONLINEAR);
        cfg.setMaxMachForAerodynamicModel(6.0);
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(cfg);

        double force = aero.getAirbrakeDeltaDragN(5.5, 1.0);

        assertTrue(Double.isFinite(force));
        assertTrue(force >= 0.0 && force <= 60.0);
        assertEquals(GenericFunction2D.EvaluationMode.EXTRAPOLATED, aero.getLastEvaluation().mode());
    }

    @Test
    void positiveOnlyTableCannotExtrapolateIntoNegativeDeltaDrag() throws Exception {
        Path csv = tempDir.resolve("steep-positive.csv");
        Files.writeString(csv, """
                Mach,Deployment,Drag_N
                3,0,0
                3,1,100
                5,0,0
                5,1,30
                """);
        AirbrakeConfig cfg = configFor(csv);
        cfg.setHighMachExtrapolationMode(HighMachExtrapolationMode.BOUNDED_NONLINEAR);
        cfg.setMaxMachForAerodynamicModel(10.0);
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(cfg);

        assertEquals(0.0, aero.getAirbrakeDeltaDragN(8.0, 1.0), 0.0);
        assertEquals(GenericFunction2D.EvaluationMode.FAIL_CLOSED, aero.getLastEvaluation().mode());
    }

    @Test
    void machAboveModelLimitFailsClosed() throws Exception {
        AirbrakeConfig cfg = configFor(resource("cfd-positive-mach0-5.csv"));
        cfg.setMaxMachForAerodynamicModel(5.0);
        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(cfg);

        assertEquals(0.0, aero.getAirbrakeDeltaDragN(5.01, 1.0), 0.0);
        assertEquals(GenericFunction2D.EvaluationMode.FAIL_CLOSED, aero.getLastEvaluation().mode());
    }

    @Test
    void deploymentPercentagesAreNormalized() throws Exception {
        Path csv = tempDir.resolve("percent.csv");
        Files.writeString(csv, """
                Mach,DeploymentPercentage,Drag_N
                0,0,0
                0,100,0
                2,0,0
                2,50,45
                2,100,90
                """);

        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(configFor(csv));

        assertEquals(45.0, aero.getAirbrakeDeltaDragN(2.0, 0.5), 1e-6);
    }

    @Test
    void missingForceColumnIsRejected() throws Exception {
        Path csv = tempDir.resolve("missing-force.csv");
        Files.writeString(csv, """
                Mach,Deployment,Cd
                0,0,0
                2,1,1
                """);

        assertThrows(IllegalArgumentException.class, () -> new AirbrakeAerodynamics(configFor(csv)));
    }

    @Test
    void nonFiniteRowsAreRejected() throws Exception {
        Path csv = tempDir.resolve("nan.csv");
        Files.writeString(csv, """
                Mach,Deployment,Drag_N
                0,0,0
                NaN,1,10
                2,0,0
                2,1,90
                """);

        assertThrows(IllegalArgumentException.class, () -> new AirbrakeAerodynamics(configFor(csv)));
    }

    @Test
    void conflictingDuplicateRowsAreRejected() throws Exception {
        Path csv = tempDir.resolve("duplicates.csv");
        Files.writeString(csv, """
                Mach,Deployment,Drag_N
                0,0,0
                0,1,0
                2,0,0
                2,1,90
                2,1,91
                """);

        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class,
                () -> new AirbrakeAerodynamics(configFor(csv)));
        assertTrue(ex.getMessage().contains("Conflicting duplicate"));
    }

    private static AirbrakeConfig configFor(Path csv) {
        AirbrakeConfig cfg = new AirbrakeConfig();
        cfg.setCfdDataFilePath(csv.toString());
        cfg.setMaxMachForDeployment(5.0);
        cfg.setMaxMachForAerodynamicModel(5.0);
        cfg.setMaxExtrapolatedForceMultiplier(2.0);
        return cfg;
    }

    private Path resource(String name) throws URISyntaxException {
        return Path.of(Objects.requireNonNull(getClass().getResource("/" + name)).toURI());
    }
}
