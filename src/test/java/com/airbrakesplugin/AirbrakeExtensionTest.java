// package com.airbrakesplugin;

// import net.sf.openrocket.simulation.SimulationConditions;
// import net.sf.openrocket.simulation.exception.SimulationException;
// import net.sf.openrocket.simulation.listeners.SimulationListener;
// import net.sf.openrocket.rocketcomponent.Rocket;
// import org.junit.jupiter.api.DisplayName;
// import org.junit.jupiter.api.Test;
// import org.junit.jupiter.api.extension.ExtendWith;
// import org.mockito.Mock;
// import org.mockito.junit.jupiter.MockitoExtension;

// import java.util.ArrayList;
// import java.util.List;

// import static org.junit.jupiter.api.Assertions.*;
// import static org.mockito.Mockito.*;

// /**
//  * Unit-tests for {@link AirbrakeExtension}.
//  */
// @ExtendWith(MockitoExtension.class)
// @DisplayName("AirbrakeExtension contract tests")
// class AirbrakeExtensionTest {

//     @Mock private SimulationConditions conditions;
//     @Mock private Rocket rocket;                    // we do not need any behaviour – just a stub

//     @Test
//     @DisplayName("initialize() adds exactly one AirbrakeSimulationListener")
//     void initialize_registersListener() throws SimulationException {
//         // -- Arrange
//         List<SimulationListener> listenerBucket = new ArrayList<>();
//         when(conditions.getSimulationListenerList()).thenReturn(listenerBucket);
//         when(conditions.getRocket()).thenReturn(rocket);

//         AirbrakeExtension ext = new AirbrakeExtension();

//         // -- Act
//         ext.initialize(conditions);

//         // -- Assert
//         assertEquals(1, listenerBucket.size(), "Exactly one listener should be registered");
//         assertTrue(listenerBucket.get(0) instanceof AirbrakeSimulationListener,
//                    "The registered listener must be an AirbrakeSimulationListener");
//     }

//     @Test
//     @DisplayName("Bean properties set → get round-trip")
//     void propertyRoundTrip() {
//         AirbrakeExtension ext = new AirbrakeExtension();

//         ext.setCfdDataFilePath("table.csv");
//         ext.setReferenceArea(0.05);
//         ext.setReferenceLength(0.80);
//         ext.setMaxDeploymentRate(0.15);
//         ext.setTargetApogee(820.0);
//         ext.setDeployAltitudeThreshold(100.0);
//         ext.setMaxMachForDeployment(0.85);

//         assertAll(
//             () -> assertEquals("table.csv", ext.getCfdDataFilePath()),
//             () -> assertEquals(0.05,   ext.getReferenceArea(),        1e-12),
//             () -> assertEquals(0.80,   ext.getReferenceLength(),      1e-12),
//             () -> assertEquals(0.15,   ext.getMaxDeploymentRate(),    1e-12),
//             () -> assertEquals(820.0,  ext.getTargetApogee(),         1e-12),
//             () -> assertEquals(100.0,  ext.getDeployAltitudeThreshold(), 1e-12),
//             () -> assertEquals(0.85,   ext.getMaxMachForDeployment(), 1e-12),
//             () -> assertNotNull(ext.toString(), "toString() should never return null")
//         );
//     }
// }
