package com.airbrakesplugin; 

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;
// import net.sf.openrocket.simulation.extension.SimulationExtension; // Not directly needed here

@Plugin // Corrected annotation
public class AirbrakePluginProvider extends AbstractSimulationExtensionProvider {

    public AirbrakePluginProvider() {
        super(AirbrakeExtension.class, "6DOF Airbrake Simulation", "Extensions");
    }
}