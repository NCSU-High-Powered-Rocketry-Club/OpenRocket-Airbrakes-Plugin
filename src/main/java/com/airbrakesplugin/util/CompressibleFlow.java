// src/main/java/com/airbrakesplugin/util/CompressibleFlow.java
package com.airbrakesplugin.util;

public final class CompressibleFlow {

    private static final double GAMMA = 1.4;   // air, below ~25 km

    private CompressibleFlow() {}              // utility-class, no instances

    /** Static–to–stagnation temperature ratio T/Tt */
    public static double staticTemperatureRatio(double mach) {
        return Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach, -1.0);
    }

    /** Static–to–stagnation pressure ratio p/Pt */
    public static double staticPressureRatio(double mach) {
        return Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach,
                        -GAMMA / (GAMMA - 1));
    }

    /** Static–to–stagnation density ratio ρ/ρt */
    public static double staticDensityRatio(double mach) {
        return Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach,
                        -1.0 / (GAMMA - 1));
    }

    /** Compressible (impact) dynamic pressure qc = Pt – p */
    public static double compressibleDynamicPressure(double staticPressure,
                                                     double mach) {
        double ptOverP = Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach,
                                  GAMMA / (GAMMA - 1));
        return staticPressure * (ptOverP - 1.0);
    }

    /** Convenience: correction factor qc / q_incompressible */
    public static double dynamicPressureCorrection(double mach) {
        if (mach < 1e-3) return 1.0;                      // avoid /0
        double base = 1 + (GAMMA - 1) * 0.5 * mach * mach;
        double incompressible =
                0.5 * GAMMA * mach * mach;               // (γ/2)M²
        return (Math.pow(base, GAMMA / (GAMMA - 1)) - 1.0) / incompressible;
    }
}
