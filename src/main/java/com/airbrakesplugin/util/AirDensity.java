package com.airbrakesplugin.util;

import com.airbrakesplugin.util.CompressibleFlow;

public final class AirDensity {
    private static final double T0 = 288.15;     // K
    private static final double P0 = 101325.0;   // Pa
    private static final double L  = 0.0065;     // K/m (troposphere lapse)
    private static final double R  = 287.05;     // J/(kg·K)
    private static final double g  = 9.80665;    // m/s^2
    private static final double GAMMA = 1.4;     // air, below ~25 km
    private static final double MACH_COMP_THRESHOLD = 0.30;

    private AirDensity() {}

    /** ISA temperature (K) – simple two-layer (0–11 km, 11–20 km). */
    public static double temperatureISA(double h) {
        if (h < 0) h = 0;
        if (h <= 11000.0) {
            return T0 - L * h;
        } else {
            // 11–20 km isothermal at T11
            return T0 - L * 11000.0;
        }
    }

    /** ISA pressure (Pa) – simple two-layer (0–11 km, 11–20 km). */
    public static double pressureISA(double h) {
        if (h < 0) h = 0;
        if (h <= 11000.0) {
            final double T = temperatureISA(h);
            return P0 * Math.pow(T / T0, g / (R * L));
        } else {
            final double T11 = temperatureISA(11000.0);
            final double P11 = P0 * Math.pow(T11 / T0, g / (R * L));
            final double H   = h - 11000.0;
            return P11 * Math.exp(-g * H / (R * T11));
        }
    }

    /** ISA density (kg/m^3) from p = ρ R T (static state). */
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
     * Compressibility-aware dynamic pressure (Pa).
     * For M ≤ 0.3 uses ½ ρ V². For M > 0.3 uses qc = Pt − p from isentropic relations.
     */
    public static double dynamicPressure(double h, double v) {
        final double M = machFromV(v, h);
        if (M <= MACH_COMP_THRESHOLD) {
            return dynamicPressureIncompressible(h, v);
        }
        final double p = pressureISA(h);
        return CompressibleFlow.compressibleDynamicPressure(p, M);
    }

    /**
     * Effective density ρ_eff for use in q = ½ ρ_eff V².
     * For M ≤ 0.3, ρ_eff = ρ. For M > 0.3, ρ_eff = ρ * (qc / q_inc),
     * where qc is compressible dynamic pressure and q_inc = ½ ρ V².
     *
     * This lets the rest of your force code keep using q = ½ ρ V²
     * while still capturing compressibility above M≈0.3.
     */
    public static double rhoForDynamicPressure(double h, double mach) {
        final double rho = rhoISA(h);
        if (!Double.isFinite(mach) || mach <= MACH_COMP_THRESHOLD) {
            return rho;
        }
        final double corr = CompressibleFlow.dynamicPressureCorrection(mach);
        return rho * corr;
    }

    /** Convenience overload when you have velocity instead of Mach. */
    public static double rhoForDynamicPressureFromV(double h, double v) {
        final double M = machFromV(v, h);
        return rhoForDynamicPressure(h, M);
    }
}
