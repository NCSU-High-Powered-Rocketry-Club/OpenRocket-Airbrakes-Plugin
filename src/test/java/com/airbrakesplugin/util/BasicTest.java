package com.airbrakesplugin.util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * A simple test that verifies the test infrastructure is working correctly.
 */
public class BasicTest {
    
    @Test
    public void simpleTest() {
        System.out.println("Running simple test to verify test infrastructure");
        assertTrue(true, "This test should always pass");
    }
    
    @Test
    public void mathTest() {
        assertEquals(4, 2+2, "Basic addition should work");
        System.out.println("Math test completed successfully");
    }
}
