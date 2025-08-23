package com.airbrakesplugin.util;

public final class AirDensity {
    private static final double T0 = 288.15;     // K
    private static final double P0 = 101325.0;   // Pa
    private static final double L  = 0.0065;     // K/m (troposphere lapse)
    private static final double R  = 287.05;     // J/(kg·K)
    private static final double g  = 9.80665;    // m/s^2

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

    /** ISA density (kg/m^3). */
    public static double rhoISA(double h) {
        final double T = temperatureISA(h);
        final double p = pressureISA(h);
        return p / (R * T);
    }
}
