// package com.airbrakesplugin;

// import net.sf.openrocket.aerodynamics.AerodynamicForces;
// import net.sf.openrocket.rocketcomponent.Rocket;
// import net.sf.openrocket.simulation.FlightDataType;
// import net.sf.openrocket.simulation.SimulationStatus;
// import net.sf.openrocket.simulation.listeners.SimulationListener;
// import net.sf.openrocket.util.Coordinate;
// import net.sf.openrocket.simulation.FlightData;
// import net.sf.openrocket.simulation.FlightDataBranch;
// import org.junit.jupiter.api.DisplayName;
// import org.junit.jupiter.api.Test;
// import org.junit.jupiter.api.extension.ExtendWith;
// import org.mockito.Mock;
// import org.mockito.junit.jupiter.MockitoExtension;

// import java.lang.reflect.Field;

// import static org.junit.jupiter.api.Assertions.*;
// import static org.mockito.ArgumentMatchers.*;
// import static org.mockito.Mockito.*;

// /**
//  * Unit–tests for {@link AirbrakeSimulationListener}.  We verify only the pure “book-keeping” logic
//  * (rate-limited deployment and coefficient injection) by mocking out OpenRocket classes.
//  *
//  * NOTE: These tests assume OpenRocket’s runtime classes are on the test class-path (compileOnly
//  * for production, but testImplementation in Gradle).
//  */
// @ExtendWith(MockitoExtension.class)
// @DisplayName("AirbrakeSimulationListener contract tests")
// class AirbrakeSimulationListenerTest {
//     // ---------- Common fixtures ----------
//     @Mock Rocket rocket;                                    // not used by SUT logic directly
//     @Mock SimulationStatus status;
//     @Mock FlightDataBranch flightData;
//     @Mock AerodynamicForces forces;

//     private static AirbrakeConfig minimalConfig() {
//         AirbrakeConfig c = new AirbrakeConfig();
//         c.setCfdDataFilePath("dummy.csv");  // anything non-blank to satisfy ctor
//         c.setMaxDeploymentRate(0.5);        // 0.5 per second for rate-limit test
//         return c;
//     }

//     // ---------- Helper to sneak mocks into private fields ----------
//     private static void inject(Object target, String fieldName, Object value) throws Exception {
//         Field f = target.getClass().getDeclaredField(fieldName);
//         f.setAccessible(true);
//         f.set(target, value);
//     }

//     // ---------- Test 1: rate-limited deployment in preStep ----------
//     @Test
//     @DisplayName("preStep() honours max deployment rate")
//     void preStep_rateLimitRespected() throws Exception {
//         // Arrange -----------------------------------------------------------------
//         AirbrakeSimulationListener listener =
//                 new AirbrakeSimulationListener(minimalConfig(), rocket);

//         // Mock controller that always asks for full deployment
//         AirbrakeController controller = mock(AirbrakeController.class);
//         when(controller.getCommandedDeployment(anyDouble(), anyDouble(), anyDouble(),
//                                                anyDouble(), any(SimulationStatus.class)))
//              .thenReturn(1.0);

//         // Provide a stub aerodynamics (not used in preStep but required for null-guard)
//         AirbrakeAerodynamics aero = mock(AirbrakeAerodynamics.class);

//         inject(listener, "controller",   controller);
//         inject(listener, "aerodynamics", aero);
//         inject(listener, "previousTime", 0.0);              // simulation starts at t = 0

//         // SimulationStatus:  Δt = 1 s  -------------------------------------------
//         when(status.getSimulationTime()).thenReturn(1.0);
//         when(status.getFlightData()).thenReturn(flightData);
//         when(flightData.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(0.3);
//         when(status.getRocketVelocity()).thenReturn(new net.sf.openrocket.util.Coordinate(0,0,100));
//         when(status.getRocketPosition()).thenReturn(new net.sf.openrocket.util.Coordinate(0,0,50));

//         // Act ---------------------------------------------------------------------
//         listener.preStep(status);

//         // Assert rate-limit:  maxRate(0.5/s) * dt(1 s)  => 0.5 expected deployment
//         Field current = listener.getClass().getDeclaredField("currentDeployment");
//         current.setAccessible(true);
//         double actualDeployment = current.getDouble(listener);

//         assertEquals(0.5, actualDeployment, 1e-12,
//                      "Deployment should be clamped to maxRate * dt");
//     }

//     // ---------- Test 2: coefficient injection in postAerodynamicCalculation ----------
//     @Test
//     @DisplayName("postAerodynamicCalculation adds ΔCᴅ and ΔCᴍ to forces")
//     void postAeroCalculation_appliesCoefficients() throws Exception {
//         // Arrange -----------------------------------------------------------------
//         AirbrakeSimulationListener listener =
//                 new AirbrakeSimulationListener(minimalConfig(), rocket);

//         // Inject ready-to-go mocks
//         AirbrakeAerodynamics aero = mock(AirbrakeAerodynamics.class);
//         when(aero.getIncrementalCd(anyDouble(), anyDouble())).thenReturn(0.03);
//         when(aero.getIncrementalCm(anyDouble(), anyDouble())).thenReturn(0.01);

//         inject(listener, "aerodynamics", aero);
//         inject(listener, "currentDeployment", 1.0);   // ensure > 1e-3 so logic executes

//         // Baseline coefficients
//         when(forces.getCD()).thenReturn(0.30);
//         when(forces.getCm()).thenReturn(0.05);

//         // Status → only Mach needed here
//         when(status.getFlightData()).thenReturn(flightData);
//         when(flightData.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(0.7);

//         // Act ---------------------------------------------------------------------
//         AerodynamicForces returned = listener.postAerodynamicCalculation(status, forces);

//         // Assert ------------------------------------------------------------------
//         assertSame(forces, returned, "Should return the *same* AerodynamicForces instance");

//         verify(forces).setCD(0.30 + 0.03);   // 0.33 expected
//         verify(forces).setCm(0.05 + 0.01);   // 0.06 expected
//     }

//     // ---------- Test 3: NaN safeguard ----------
//     @Test
//     @DisplayName("postAerodynamicCalculation skips update when CFD returns NaN")
//     void postAeroCalculation_skipsOnNaN() throws Exception {
//         AirbrakeSimulationListener listener =
//                 new AirbrakeSimulationListener(minimalConfig(), rocket);

//         AirbrakeAerodynamics aero = mock(AirbrakeAerodynamics.class);
//         when(aero.getIncrementalCd(anyDouble(), anyDouble())).thenReturn(Double.NaN);
//         when(aero.getIncrementalCm(anyDouble(), anyDouble())).thenReturn(0.02);

//         inject(listener, "aerodynamics", aero);
//         inject(listener, "currentDeployment", 0.8);

//         when(status.getFlightData()).thenReturn(flightData);
//         when(flightData.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(1.2);

//         listener.postAerodynamicCalculation(status, forces);

//         // Neither coefficient setter should be invoked
//         verify(forces, never()).setCD(anyDouble());
//         verify(forces, never()).setCm(anyDouble());
//     }
//     // @Test
//     // void dimensionalForceIsAdded() throws Exception {
//     //     AirbrakeSimulationListener sut =
//     //             new AirbrakeSimulationListener(minimalConfig(), rocket);

//     //     inject(sut, "currentDeployment", 1.0);
//     //     AirbrakeAerodynamics aero = mock(AirbrakeAerodynamics.class);
//     //     when(aero.getIncrementalCd(anyDouble(), anyDouble())).thenReturn(0.25); // big drag
//     //     when(aero.getIncrementalCm(anyDouble(), anyDouble())).thenReturn(0.0);
//     //     inject(sut, "aerodynamics", aero);

//     //     when(status.getFlightData()).thenReturn(flightData);
//     //     when(flightData.getLast(FlightDataType.TYPE_MACH_NUMBER)).thenReturn(0.5);

//     //     Coordinate velocity = new Coordinate(0,0,150);    // 150 m/s straight up
//     //     when(status.getRocketVelocity()).thenReturn(velocity);
//     //     when(status.getRocketPosition()).thenReturn(new Coordinate(0,0,500));

//     //     Coordinate baseF = new Coordinate(0,0,-40);       // 40 N baseline drag
//     //     Coordinate baseM = new Coordinate();
//     //     AerodynamicForces base = new AerodynamicForces();
//     //     base.setCm(0.2);;
//     //     // No direct setMoment method available in AerodynamicForces
//     //     base.setCD(0.5);

//     //     AerodynamicForces out = sut.postAerodynamicCalculation(status, base);

//     //     // magnitude must increase:
//     //     assertTrue(out.getCN().length() > baseF.length() + 10.0);
//     // }

// }
