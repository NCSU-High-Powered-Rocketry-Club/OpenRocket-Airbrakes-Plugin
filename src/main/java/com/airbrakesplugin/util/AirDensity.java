package com.airbrakesplugin.util;

/**
 * Air density calculations
 */
public final class AirDensity {
    private AirDensity() {
    }

    /**
     * Returns the air density at a given altitude
     * Same method used for PyAnsys simulations, verified to be correct
     *
     * @param altitude (m)
     * @return air density (kg/m^3)
     */
    public static double getAirDensityAtAltitude(double altitude) {
        // Based on US Standard Atmosphere 1976
        double temperature, pressure, density;

        if (altitude <= 11000) {  // Troposphere
            temperature = 288.15 - 0.00649 * altitude;
            pressure = 101325 * Math.pow((1 - 0.00649 * altitude / 288.15), 5.2561);
        } else if (altitude <= 25000) {  // Stratosphere
            temperature = 216.65;
            pressure = 22632.06 * Math.exp(-0.000157 * (altitude - 11000));
        } else {  // Mesosphere
            temperature = 273.15 - 0.0028 * altitude;
            pressure = 5474.89 * Math.pow((1 - 0.0028 * altitude / 273.15), 5.2561);
        }

        density = pressure / (287.05 * temperature);

        return density;
    }
    public static double getStaticPressureAtAltitude(double altitude) {
        final double P0 = 101_325.0;      // sea-level pressure, Pa
        final double T0 = 288.15;         // sea-level temperature, K
        final double L  = 0.0065;         // lapse rate, K / m
        final double g  = 9.80665;        // m / s²
        final double R  = 287.05;         // J / kg · K

        if (altitude < 11_000.0) {        // troposphere
            double T = T0 - L * altitude;
            return P0 * Math.pow(T / T0, g / (R * L));
        } else {                          // 11-25 km isothermal layer
            double p11 = 22_632.06;       // pressure at 11 km, Pa
            double T11 = 216.65;          // temperature at 11 km, K
            return p11 * Math.exp(-g * (altitude - 11_000.0) /
                                (R * T11));
        }
    }
}

