// src/main/java/com/airbrakesplugin/util/CompressibleFlow.java
package com.airbrakesplugin.util;

public final class CompressibleFlow {

    public static final double GAMMA = 1.4;   // air, below ~25 km
    public static final double R_AIR = 287.05;

    private CompressibleFlow() {}              // utility-class, no instances

    public static double speedOfSound(double staticTemperatureK) {
        if (!Double.isFinite(staticTemperatureK) || staticTemperatureK <= 0.0) {
            throw new IllegalArgumentException("staticTemperatureK must be finite and positive");
        }
        return Math.sqrt(GAMMA * R_AIR * staticTemperatureK);
    }

    public static double machFromVelocity(double trueAirspeedMps, double staticTemperatureK) {
        if (!Double.isFinite(trueAirspeedMps) || trueAirspeedMps < 0.0) {
            throw new IllegalArgumentException("trueAirspeedMps must be finite and non-negative");
        }
        return trueAirspeedMps / speedOfSound(staticTemperatureK);
    }

    public static double dynamicPressure(double densityKgM3, double trueAirspeedMps) {
        if (!Double.isFinite(densityKgM3) || densityKgM3 < 0.0) {
            throw new IllegalArgumentException("densityKgM3 must be finite and non-negative");
        }
        if (!Double.isFinite(trueAirspeedMps) || trueAirspeedMps < 0.0) {
            throw new IllegalArgumentException("trueAirspeedMps must be finite and non-negative");
        }
        return 0.5 * densityKgM3 * trueAirspeedMps * trueAirspeedMps;
    }

    /** Static–to–stagnation temperature ratio T/Tt */
    public static double staticTemperatureRatio(double mach) {
        requireSubsonicIsentropicMach(mach);
        return Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach, -1.0);
    }

    /** Static–to–stagnation pressure ratio p/Pt */
    public static double staticPressureRatio(double mach) {
        requireSubsonicIsentropicMach(mach);
        return Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach, -GAMMA / (GAMMA - 1));
    }

    /** Static–to–stagnation density ratio ρ/ρt */
    public static double staticDensityRatio(double mach) {
        requireSubsonicIsentropicMach(mach);
        return Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach, -1.0 / (GAMMA - 1));
    }

    /** Subsonic isentropic impact pressure qc = Pt - p. Throws for Mach >= 1. */
    public static double compressibleDynamicPressure(double staticPressure, double mach) {
        requireSubsonicIsentropicMach(mach);
        return subsonicImpactPressure(staticPressure, mach);
    }

    public static double subsonicImpactPressure(double staticPressure, double mach) {
        requirePressure(staticPressure);
        requireSubsonicIsentropicMach(mach);
        double ptOverP = Math.pow(1 + (GAMMA - 1) * 0.5 * mach * mach, GAMMA / (GAMMA - 1));
        return staticPressure * (ptOverP - 1.0);
    }

    /**
     * Rayleigh pitot impact pressure for supersonic flow behind a normal shock.
     * This is an explicit supersonic helper and should not be used as local
     * aerodynamic q for force-to-Cd conversion.
     */
    public static double supersonicPitotImpactPressure(double staticPressure, double mach) {
        requirePressure(staticPressure);
        if (!Double.isFinite(mach) || mach < 1.0) {
            throw new IllegalArgumentException("mach must be finite and >= 1 for supersonic pitot pressure");
        }
        double gm1 = GAMMA - 1.0;
        double gp1 = GAMMA + 1.0;
        double m2 = mach * mach;
        double normalShockStaticRatio = (2.0 * GAMMA * m2 - gm1) / gp1;
        double m2BehindShock = (1.0 + 0.5 * gm1 * m2) / (GAMMA * m2 - 0.5 * gm1);
        double p02OverP2 = Math.pow(1.0 + 0.5 * gm1 * m2BehindShock, GAMMA / gm1);
        double p02OverP1 = normalShockStaticRatio * p02OverP2;
        return staticPressure * (p02OverP1 - 1.0);
    }

    /** Convenience: subsonic correction factor qc / q_incompressible. Throws for Mach >= 1. */
    public static double dynamicPressureCorrection(double mach) {
        requireSubsonicIsentropicMach(mach);
        if (mach < 1e-3) return 1.0;                      // avoid /0
        double base = 1 + (GAMMA - 1) * 0.5 * mach * mach;
        double incompressible = 0.5 * GAMMA * mach * mach;               // (γ/2)M²
        return (Math.pow(base, GAMMA / (GAMMA - 1)) - 1.0) / incompressible;
    }

    private static void requirePressure(double staticPressure) {
        if (!Double.isFinite(staticPressure) || staticPressure < 0.0) {
            throw new IllegalArgumentException("staticPressure must be finite and non-negative");
        }
    }

    private static void requireSubsonicIsentropicMach(double mach) {
        if (!Double.isFinite(mach) || mach < 0.0 || mach >= 1.0) {
            throw new IllegalArgumentException("mach must be finite and in [0, 1) for subsonic isentropic relations");
        }
    }
}
