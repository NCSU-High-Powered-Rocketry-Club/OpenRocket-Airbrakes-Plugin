package com.airbrakesplugin;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;

import static org.junit.jupiter.api.Assertions.*;

@DisplayName("AirbrakeController basic functionality tests")
class AirbrakeControllerTest {
    
    private AirbrakeController controller;
    private AirbrakeConfig config;
    
    @BeforeEach
    void setup() {
        config = new AirbrakeConfig();
        config.setTargetApogee(1000.0);
        controller = new AirbrakeController(config);
    }

    @Test
    @DisplayName("Should return zero deployment when vZ is zero or negative")
    void zeroDeployment_whenNotAscending() {
        // Act & Assert
        // Using null for SimulationStatus which the controller handles safely
        double resultZero = controller.getCommandedDeployment(500, 0.0, 0.3, 0.0, null);
        double resultNegative = controller.getCommandedDeployment(500, -1.0, 0.3, 0.0, null);
        
        assertEquals(0.0, resultZero, 0.001, "Should be zero when vZ is zero");
        assertEquals(0.0, resultNegative, 0.001, "Should be zero when vZ is negative");
    }
    
    @Test
    @DisplayName("Should return fixed percentage when in always-open mode")
    void fixedDeployment_whenAlwaysOpenMode() {
        // Arrange
        config.setAlwaysOpenMode(true);
        config.setAlwaysOpenPercent(0.75);
        controller = new AirbrakeController(config);
        
        // Act - Using null for SimulationStatus which the controller handles safely
        double deployment = controller.getCommandedDeployment(500, 10.0, 0.3, 0.0, null);
        
        // Assert
        assertEquals(0.75, deployment, 0.001, "Should return the fixed percentage");
    }
}
