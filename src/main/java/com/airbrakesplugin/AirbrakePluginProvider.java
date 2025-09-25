package com.airbrakesplugin; 

import info.openrocket.core.plugin.Plugin;
import info.openrocket.core.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin 
public class AirbrakePluginProvider extends AbstractSimulationExtensionProvider {

    public AirbrakePluginProvider() {
        super(AirbrakeExtension.class, "HPRC Airbrakes Plugin", "Extensions");
    }
}