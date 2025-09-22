package com.airbrakesplugin; 

import info.openrocket.core.plugin.Plugin;
import info.openrocket.core.simulation.extension.AbstractSimulationExtensionProvider;
// import net.sf.openrocket.simulation.extension.SimulationExtension; // Not directly needed here

@Plugin // Corrected annotation
public class AirbrakePluginProvider extends AbstractSimulationExtensionProvider {

    public AirbrakePluginProvider() {
        super(AirbrakeExtension.class, "HPRC Airbrakes Plugin", "Extensions");
    }
}