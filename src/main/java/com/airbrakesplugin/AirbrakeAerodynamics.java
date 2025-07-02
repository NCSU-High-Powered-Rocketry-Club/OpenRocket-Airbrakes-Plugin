package com.airbrakesplugin;

import com.airbrakesplugin.util.GenericFunction2D;
import java.nio.file.Paths;

public class AirbrakeAerodynamics {

    private final GenericFunction2D surface;

    public AirbrakeAerodynamics(String csvFilePath) {
        if (csvFilePath == null || csvFilePath.isBlank())
            throw new IllegalArgumentException("CSV file path is null/empty");

        surface = new GenericFunction2D(Paths.get(csvFilePath));
    }

    public double getIncrementalCd(double mach, double deployFrac) {
        return surface.valueCd(mach, deployFrac);
    }
    public double getIncrementalCm(double mach, double deployFrac) {
        return surface.valueCm(mach, deployFrac);
    }
}
