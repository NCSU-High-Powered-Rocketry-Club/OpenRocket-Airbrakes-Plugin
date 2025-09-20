// // src/test/java/com/airbrakesplugin/AirbrakeControllerSetpointPersistenceTest.java
// package com.airbrakesplugin;

// import com.airbrakesplugin.util.ApogeePredictor;
// import info.openrocket.core.simulation.SimulationStatus;
// import org.junit.jupiter.api.Test;
// import static org.mockito.Mockito.*;
// import static org.junit.jupiter.api.Assertions.*;

// public class AirbrakeControllerSetpointPersistenceTest {
//     @Test
//     public void setpointPersistsAcrossNoChangeTicks() {
//         ApogeePredictor pred = mock(ApogeePredictor.class);
//         SimulationStatus st = mock(SimulationStatus.class);
//         AirbrakeController ctrl = new AirbrakeController(100.0, pred, null, 0.0, 5.0, 2.0);

//         when(st.getSimulationTime()).thenReturn(0.0, 0.1, 0.2, 0.3);
//         when(pred.getPredictionIfReady())
//                 .thenReturn(120.0) // extend
//                 .thenReturn(101.0) // inside db: no change (stay extend)
//                 .thenReturn(99.5)  // inside db: no change (stay extend)
//                 .thenReturn(98.0); // retract (cross -db)

//         assertTrue(ctrl.updateAndGateFlexible(st));
//         assertTrue(ctrl.hasSetpoint());
//         assertEquals(1.0, ctrl.currentSetpoint(), 1e-12);

//         assertFalse(ctrl.updateAndGateFlexible(st));
//         assertEquals(1.0, ctrl.currentSetpoint(), 1e-12);

//         assertFalse(ctrl.updateAndGateFlexible(st));
//         assertEquals(1.0, ctrl.currentSetpoint(), 1e-12);

//         assertTrue(ctrl.updateAndGateFlexible(st));
//         assertEquals(0.0, ctrl.currentSetpoint(), 1e-12);
//     }
// }
