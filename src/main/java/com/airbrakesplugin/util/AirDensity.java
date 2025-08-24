// src/main/java/com/airbrakesplugin/util/AirDensity.java
package com.airbrakesplugin.util;

public final class AirDensity {
    // ISA / gas constants
    private static final double T0 = 288.15;     // K
    private static final double P0 = 101_325.0;  // Pa
    private static final double L  = 0.0065;     // K/m (troposphere lapse rate)
    private static final double R  = 287.05;     // J/(kg·K)
    private static final double g  = 9.80665;    // m/s^2
    private static final double GAMMA = 1.4;     // air, up to ~25 km

    // Compressibility handling
    private static final double MACH_COMP_THRESHOLD = 0.30;

    private AirDensity() {} // utility class

    /** ISA temperature (K) – simple two-layer (0–11 km, 11–20 km isothermal). */
    public static double temperatureISA(double h) {
        if (h < 0) h = 0;
        if (h <= 11_000.0) {
            return T0 - L * h;
        } else {
            // 11–20 km isothermal at T11
            return T0 - L * 11_000.0;
        }
    }

    /** ISA pressure (Pa) – simple two-layer (0–11 km, 11–20 km isothermal). */
    public static double pressureISA(double h) {
        if (h < 0) h = 0;

        if (h <= 11_000.0) {
            final double T = temperatureISA(h);
            // Barometric formula with linear lapse: p = P0 * (T/T0)^(g/(L*R))
            return P0 * Math.pow(T / T0, g / (R * L));
        } else {
            final double T11 = temperatureISA(11_000.0);
            final double P11 = P0 * Math.pow(T11 / T0, g / (R * L));
            final double H   = h - 11_000.0;
            // Isothermal layer: p = P11 * exp(-g*Δh/(R*T11))
            return P11 * Math.exp(-g * H / (R * T11));
        }
    }

    /** ISA density (kg/m^3) via ideal gas law. */
    public static double rhoISA(double h) {
        final double T = temperatureISA(h);
        final double p = pressureISA(h);
        return p / (R * T);
    }

    /** ISA speed of sound a = sqrt(γ R T) (m/s). */
    public static double speedOfSoundISA(double h) {
        final double T = temperatureISA(h);
        return Math.sqrt(GAMMA * R * T);
    }

    /** Mach from true airspeed (m/s) and altitude (m). */
    public static double machFromV(double v, double h) {
        final double a = speedOfSoundISA(h);
        return (a > 1e-9) ? (v / a) : 0.0;
    }

    /** Incompressible dynamic pressure q_inc = ½ ρ V² (Pa). */
    public static double dynamicPressureIncompressible(double h, double v) {
        final double rho = rhoISA(h);
        return 0.5 * rho * v * v;
    }

    /**
     * Compressibility-aware dynamic pressure using velocity (Pa).
     * For M ≤ 0.3 uses q_inc = ½ ρ V².
     * For M > 0.3 uses isentropic impact pressure: qc = Pt − p.
     */
    public static double dynamicPressure(double h, double v) {
        final double M = machFromV(v, h);
        return dynamicPressureFromMach(h, M, v);
    }

    /**
     * Compressibility-aware dynamic pressure when Mach is already known (Pa).
     * If M ≤ 0.3 returns ½ ρ V² (requires V).
     * If M > 0.3 returns qc = Pt − p (velocity not needed in that branch).
     */
    public static double dynamicPressureFromMach(double h, double mach, double vIfNeeded) {
        if (!Double.isFinite(mach) || mach <= MACH_COMP_THRESHOLD) {
            return dynamicPressureIncompressible(h, vIfNeeded);
        }
        final double p = pressureISA(h);
        return CompressibleFlow.compressibleDynamicPressure(p, mach);
    }

    /**
     * Effective density ρ_eff for use in q = ½ ρ_eff V².
     * For M ≤ 0.3, ρ_eff = ρ.
     * For M > 0.3, ρ_eff = ρ * (qc / q_inc), where
     *    q_inc = ½ ρ V² and qc = Pt − p from isentropic relations.
     */
    public static double rhoForDynamicPressure(double h, double mach) {
        final double rho = rhoISA(h);
        if (!Double.isFinite(mach) || mach <= MACH_COMP_THRESHOLD) {
            return rho;
        }
        final double corr = CompressibleFlow.dynamicPressureCorrection(mach); // qc / q_inc
        return rho * corr;
    }

    /** Convenience overload when you have velocity instead of Mach. */
    public static double rhoForDynamicPressureFromV(double h, double v) {
        final double M = machFromV(v, h);
        return rhoForDynamicPressure(h, M);
    }
}
