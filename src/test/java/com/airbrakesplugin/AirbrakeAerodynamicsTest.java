package com.airbrakesplugin;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

@DisplayName("AirbrakeAerodynamics basic contract tests")
class AirbrakeAerodynamicsTest {

    @Test
    @DisplayName("Constructor rejects null path")
    void constructor_nullPath_throws() {
        assertThrows(IllegalArgumentException.class,
                     () -> new AirbrakeAerodynamics(null));
    }

    @Test
    @DisplayName("Constructor rejects blank path")
    void constructor_blankPath_throws() {
        assertThrows(IllegalArgumentException.class,
                     () -> new AirbrakeAerodynamics("   "));
    }

    @Test
    @DisplayName("Loads CSV and echoes grid-point values correctly")
    void lookup_returnsExpectedValues(@TempDir Path tmp) throws IOException {
        // --- Arrange – create a minimal CSV the Prod code can ingest
        Path csv = tmp.resolve("airbrake-cfd.csv");
        String csvContent =
                "Mach,DeploymentPercentage,Cd_increment,Cm_increment\n" +
                "0.30,0.00,0.000,0.000\n" +
                "0.30,1.00,0.100,0.020\n" +
                "1.00,0.00,0.200,0.050\n" +
                "1.00,1.00,0.300,0.080\n";
        Files.write(csv, csvContent.getBytes(StandardCharsets.UTF_8));

        AirbrakeAerodynamics aero = new AirbrakeAerodynamics(csv.toString());

        // --- Act & Assert – exact grid-point queries
        assertAll(
            () -> assertEquals(0.000, aero.getIncrementalCd(0.30, 0.00), 1e-9),
            () -> assertEquals(0.000, aero.getIncrementalCm(0.30, 0.00), 1e-9),

            () -> assertEquals(0.100, aero.getIncrementalCd(0.30, 1.00), 1e-9),
            () -> assertEquals(0.020, aero.getIncrementalCm(0.30, 1.00), 1e-9),

            () -> assertEquals(0.200, aero.getIncrementalCd(1.00, 0.00), 1e-9),
            () -> assertEquals(0.050, aero.getIncrementalCm(1.00, 0.00), 1e-9),

            () -> assertEquals(0.300, aero.getIncrementalCd(1.00, 1.00), 1e-9),
            () -> assertEquals(0.080, aero.getIncrementalCm(1.00, 1.00), 1e-9)
        );
    }
}
