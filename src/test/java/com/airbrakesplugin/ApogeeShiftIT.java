// @RunWith(Parameterized.class)            // run once w/ plugin, once w/o
// public class ApogeeShiftIT {

//     // Parameters: boolean usePlugin
//     @Parameter(0) public boolean pluginEnabled;

//     @Test
//     public void pluginChangesApogee() throws Exception {
//         Path orkFile   = Paths.get("");        // <-- supply .ork path
//         Path cfdCsv    = Paths.get("");        // <-- supply CSV path

//         OpenRocketInstance or = OpenRocketInstance.launchHeadless();
//         Simulation          s = or.loadDesign(orkFile).getSimulation(0);

//         if (pluginEnabled) {
//             AirbrakeExtension ext = new AirbrakeExtension();
//             ext.setCfdDataFilePath(cfdCsv.toString());
//             s.addSimulationExtension(ext);
//         }

//         FlightData data = s.simulate().getFlightData();
//         double apogee  = data.max(FlightDataType.TYPE_ALTITUDE);

//         // store for later comparison …
//         recordApogee(pluginEnabled, apogee);
//     }

//     @AfterClass
//     public static void compare() {
//         assertTrue(APOGEE_WITH_PLUGIN < APOGEE_BASELINE - 5.0,
//                    "Airbrakes should lower apogee by at least 5 m");
//     }
// }
