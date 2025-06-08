package com.airbrakesplugin;

import com.airbrakesplugin.util.Function2D;
import com.airbrakesplugin.util.GenericFunction2D;
import com.airbrakesplugin.util.ExtrapolationType;

import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Wraps two GenericFunction2D instances for ΔC_D and ΔC_M interpolation.
 */
public class AirbrakeAerodynamics {

    private final Function2D cdFunction;
    private final Function2D cmFunction;

    /**
     * Constructs an AirbrakeAerodynamics using CFD data CSV.
     * @param csvFilePath path to CSV with headers Mach, DeploymentPercentage, Cd_increment, Cm_increment
     */
    public AirbrakeAerodynamics(String csvFilePath) {
        if (csvFilePath == null || csvFilePath.trim().isEmpty()) {
            throw new IllegalArgumentException("CSV file path cannot be null or empty.");
        }
        Path path = Paths.get(csvFilePath);
        // initialize with constant extrapolation; adjust if needed
        this.cdFunction = new GenericFunction2D(path, "Cd_increment", ExtrapolationType.CONSTANT);
        this.cmFunction = new GenericFunction2D(path, "Cm_increment", ExtrapolationType.CONSTANT);
    }

    /**
     * @param mach Mach number
     * @param deploymentFraction deployment fraction [0,1]
     * @return incremental drag coefficient ΔC_D
     */
    public double getIncrementalCd(double mach, double deploymentFraction) {
        return cdFunction.value(mach, deploymentFraction);
    }

    /**
     * @param mach Mach number
     * @param deploymentFraction deployment fraction [0,1]
     * @return incremental moment coefficient ΔC_M
     */
    public double getIncrementalCm(double mach, double deploymentFraction) {
        return cmFunction.value(mach, deploymentFraction);
    }
}
