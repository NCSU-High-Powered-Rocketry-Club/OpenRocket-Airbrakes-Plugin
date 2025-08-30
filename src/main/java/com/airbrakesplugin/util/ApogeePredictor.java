package com.airbrakesplugin.util;

// package com.airbrakesplugin.predictor;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

/**
 * ApogeePredictor — Java port of the Python predictor (no internal gating).
 *
 * Usage contract (matches Python):
 *  - Call update(...) ONLY during coast phase, passing:
 *      verticalAcceleration  : acceleration along your chosen "up" axis (includes gravity!)
 *      dt                    : time since last sample [s]
 *      currentAltitude       : current altitude [m]
 *      currentVelocity       : current velocity along the same axis [m/s]
 *  - The predictor will buffer samples; when enough samples have arrived it
 *    runs the curve fit, updates the LUT, and (once converged) provides a prediction.
 */
public final class ApogeePredictor {

    // ===== Python-analog constants (tune these to match your config) =====
    private final int    APOGEE_PREDICTION_MIN_PACKETS;   // Python: min new samples between fits
    private final double FLIGHT_LENGTH_SECONDS;
    private final double INTEGRATION_TIME_STEP_SECONDS;
    private final double GRAVITY_MPS2;
    private final double UNCERTAINTY_THRESHOLD;
    private final double INIT_A;
    private final double INIT_B;

    // ===== State =====
    private final Deque<Double> accelerations = new ArrayDeque<>(); // raw accel (WITH gravity), coast only
    private final Deque<Double> dts           = new ArrayDeque<>(); // dt samples
    private double[] cumulativeTime = new double[0];

    private double currentAltitude = 0.0;       // latest altitude
    private double currentVelocity = 0.0;       // latest velocity (axis = same as acceleration)
    private Double initialVelocity = null;      // latched once when LUT first built

    private boolean hasConverged = false;
    private int     lastRunLength = 0;

    // curve coefficients + uncertainties
    private double A = 0.0, B = 0.0;
    private double[] uncertainties = new double[] {0.0, 0.0};

    // lookup table (ascending velocities)
    private double[] lutVelocities   = new double[] {0.0, 0.1};
    private double[] lutDeltaHeights = new double[] {0.1, 0.1};

    // cached timestamps for LUT integration
    private final double[] PREDICTED_TIMES;

    public ApogeePredictor(
            int    minPackets,
            double flightLenSeconds,
            double integrationDt,
            double gravity,
            double uncertaintyThreshold,
            double initA,
            double initB
    ) {
        this.APOGEE_PREDICTION_MIN_PACKETS  = minPackets;
        this.FLIGHT_LENGTH_SECONDS          = flightLenSeconds;
        this.INTEGRATION_TIME_STEP_SECONDS  = integrationDt;
        this.GRAVITY_MPS2                   = gravity;
        this.UNCERTAINTY_THRESHOLD          = uncertaintyThreshold;
        this.INIT_A                         = initA;
        this.INIT_B                         = initB;
        // build time grid like the Python np.arange(0, FLIGHT_LENGTH_SECONDS, INTEGRATION_DT)
        int steps = Math.max(1, (int)Math.ceil(flightLenSeconds / integrationDt));
        this.PREDICTED_TIMES = new double[steps];
        for (int i = 0; i < steps; i++) PREDICTED_TIMES[i] = i * integrationDt;
    }

    /** Convenience ctor with sensible defaults; override via your config if needed. */
    public static ApogeePredictor defaultConfig() {
        return new ApogeePredictor(
                /*minPackets*/ 8,
                /*flightLen*/ 120.0,
                /*dt*/        0.01,
                /*g*/         9.80665,
                /*uncThr*/    0.1,
                /*A0*/        1.0,
                /*B0*/        0.05
        );
    }

    /** True when the curve fit has converged (uncertainties below threshold). */
    public boolean hasConverged() { return hasConverged; }

    /** Latest coefficients and 1σ uncertainties (for logging/telemetry). */
    public double getA() { return A; }
    public double getB() { return B; }
    public double getSigmaA() { return uncertainties[0]; }
    public double getSigmaB() { return uncertainties[1]; }

    /** Number of buffered samples (coast only). */
    public int sampleCount() { return accelerations.size(); }

    /**
     * Feed one coast-phase sample (Python parity: called only during coast).
     * @param verticalAcceleration acceleration along the chosen axis, INCLUDING gravity [m/s^2]
     * @param dt                   time since last sample [s]
     * @param currentAltitude      current altitude [m]
     * @param currentVelocity      current velocity along the same axis [m/s]
     */
    public void update(double verticalAcceleration, double dt,
                       double currentAltitude, double currentVelocity) {
        // Append new sample
        accelerations.addLast(verticalAcceleration);
        dts.addLast(Math.max(1e-6, dt)); // robust dt
        this.currentAltitude = currentAltitude;
        this.currentVelocity = currentVelocity;

        // Only run the heavy stuff if we've received enough new samples since last try
        if ((accelerations.size() - lastRunLength) >= APOGEE_PREDICTION_MIN_PACKETS) {
            // update cumulative times (like np.cumsum)
            cumulativeTime = cumsum(toArray(dts));

            // If not converged, run a curve fit and update the LUT
            if (!hasConverged) {
                CurveCoefficients cc = curveFit(cumulativeTime, toArray(accelerations), INIT_A, INIT_B);
                this.A = cc.A;
                this.B = cc.B;
                this.uncertainties = cc.uncertainties.clone();
                if (uncertainties[0] < UNCERTAINTY_THRESHOLD && uncertainties[1] < UNCERTAINTY_THRESHOLD) {
                    hasConverged = true;
                }
                // Rebuild LUT from the latest coefficients (Python does this each run)
                updatePredictionLookupTable(this.A, this.B);
            }

            // Record how many samples were processed this run
            lastRunLength = accelerations.size();
        }
    }

    /**
     * Returns the predicted apogee if (and only if) the fit has converged;
     * otherwise returns null (Python only emits after convergence).
     */
    public Double getPredictionIfReady() {
        if (!hasConverged) return null;
        // ΔH from LUT at current velocity (interpolated) + current altitude
        final double dH = interp1(currentVelocity, lutVelocities, lutDeltaHeights);
        return currentAltitude + dH;
    }

    // ================================ Python-parity internals ================================

    /** Model function a(t) = A * (1 - B t)^4 */
    private static double modelAccel(double t, double A, double B) {
        final double u = 1.0 - B * t;
        return A * u * u * u * u;
    }

    /**
     * Curve fit to a(t) = A (1 - B t)^4 using a tiny LM/Gauss-Newton
     * and covariance ≈ σ² (JᵀJ)⁻¹ (Python's curve_fit analogue).
     */
    private CurveCoefficients curveFit(double[] t, double[] y, double A0, double B0) {
        double A = A0, B = B0;
        double lambda = 1e-3;
        final int maxIter = 2000;

        double prevRSS = Double.POSITIVE_INFINITY;

        for (int it = 0; it < maxIter; it++) {
            double j11=0, j12=0, j22=0, r1=0, r2=0, rss=0;

            for (int i = 0; i < t.length; i++) {
                final double ti = t[i];
                final double u  = 1.0 - B * ti;
                final double u2 = u*u, u3=u2*u, u4=u3*u;
                final double f  = A * u4;
                final double ri = y[i] - f;

                final double dA = u4;
                final double dB = A * 4.0 * u3 * (-ti);

                j11 += dA*dA;
                j12 += dA*dB;
                j22 += dB*dB;
                r1  += dA*ri;
                r2  += dB*ri;
                rss += ri*ri;
            }

            // Levenberg–Marquardt step on 2x2 normal equations
            final double d = (j11 + lambda) * (j22 + lambda) - j12 * j12;
            if (Math.abs(d) < 1e-16) break;

            final double dAstep = ((j22 + lambda) * r1 - j12 * r2) / d;
            final double dBstep = (-(j12) * r1 + (j11 + lambda) * r2) / d;

            final double Anew = A + dAstep;
            final double Bnew = B + dBstep;

            // Evaluate new RSS
            double rssNew = 0.0;
            for (int i = 0; i < t.length; i++) {
                final double ti = t[i];
                final double fi = Anew * Math.pow(1.0 - Bnew * ti, 4.0);
                final double ri = y[i] - fi;
                rssNew += ri * ri;
            }

            if (rssNew < rss) {
                A = Anew; B = Bnew;
                lambda = Math.max(1e-12, lambda * 0.33);
            } else {
                lambda = Math.min(1e6, lambda * 3.0);
            }

            if (Math.abs(dAstep) + Math.abs(dBstep) < 1e-9) break;
            if (Math.abs(prevRSS - rss) < 1e-12) break;
            prevRSS = rss;
        }

        // Covariance estimate: σ² (JᵀJ)⁻¹ with σ² = RSS / (N - 2)
        double j11=0, j12=0, j22=0, rss=0;
        for (int i = 0; i < t.length; i++) {
            final double ti = t[i];
            final double u  = 1.0 - B * ti;
            final double u2 = u*u, u3=u2*u, u4=u3*u;
            final double fi = A * u4;
            final double ri = y[i] - fi;

            final double dA = u4;
            final double dB = A * 4.0 * u3 * (-ti);

            j11 += dA*dA;
            j12 += dA*dB;
            j22 += dB*dB;
            rss += ri*ri;
        }
        final double sigma2 = rss / Math.max(1, t.length - 2);
        final double det = j11 * j22 - j12 * j12;
        double sA = Double.POSITIVE_INFINITY, sB = Double.POSITIVE_INFINITY;
        if (Math.abs(det) > 1e-16) {
            final double inv11 =  j22 / det;
            final double inv12 = -j12 / det;
            final double inv22 =  j11 / det;
            sA = Math.sqrt(Math.max(0.0, sigma2 * inv11));
            sB = Math.sqrt(Math.max(0.0, sigma2 * inv22));
        }
        return new CurveCoefficients(A, B, new double[]{sA, sB});
    }

    /**
     * Build the velocity→Δheight LUT exactly like the Python:
     * predicted_accel = A*(1 - B*t)^4 - g
     * v = v0 + cumsum(predicted_accel) * dt
     * keep v>=0; h = cumsum(v)*dt; apogee = max(h)
     * flip arrays so velocity is ascending; ΔH = apogee - h
     */
    private void updatePredictionLookupTable(final double a, final double b) {
        if (initialVelocity == null) initialVelocity = currentVelocity;

        // predicted accelerations over time (gravity subtracted here)
        final int n = PREDICTED_TIMES.length;
        final double[] aPred = new double[n];
        for (int i = 0; i < n; i++) {
            aPred[i] = modelAccel(PREDICTED_TIMES[i], a, b) - GRAVITY_MPS2;
        }

        // integrate to velocity
        final double[] v = new double[n];
        double vv = initialVelocity.doubleValue();
        for (int i = 0; i < n; i++) {
            vv += aPred[i] * INTEGRATION_TIME_STEP_SECONDS;
            v[i] = vv;
        }

        // keep only non-negative v (still ascending)
        int lastNonNeg = -1;
        for (int i = 0; i < n; i++) if (v[i] >= 0.0) lastNonNeg = i;
        if (lastNonNeg < 1) {
            // degenerate; keep a tiny, safe LUT
            lutVelocities   = new double[] {0.0, 0.1};
            lutDeltaHeights = new double[] {0.1, 0.1};
            return;
        }
        final int m = lastNonNeg + 1;
        final double[] vKeep = Arrays.copyOf(v, m);

        // integrate velocity to relative altitude
        final double[] h = new double[m];
        double hh = 0.0;
        for (int i = 0; i < m; i++) {
            hh += vKeep[i] * INTEGRATION_TIME_STEP_SECONDS;
            h[i] = hh;
        }

        // apogee
        final double apogeeRel = h[m - 1];

        // flip to ascending velocity
        reverseInPlace(vKeep);
        reverseInPlace(h);

        // ΔH = apogee - h
        final double[] dH = new double[m];
        for (int i = 0; i < m; i++) dH[i] = Math.max(0.0, apogeeRel - h[i]);

        lutVelocities   = vKeep;
        lutDeltaHeights = dH;
    }

    // ================================ utilities ================================

    private static class CurveCoefficients {
        final double A, B;
        final double[] uncertainties;
        CurveCoefficients(double A, double B, double[] uncertainties) {
            this.A = A; this.B = B; this.uncertainties = uncertainties;
        }
    }

    private static double[] toArray(Deque<Double> dq) {
        final double[] out = new double[dq.size()];
        int i = 0; for (double v : dq) out[i++] = v; return out;
    }

    private static double[] cumsum(double[] x) {
        final double[] out = new double[x.length];
        double s = 0.0;
        for (int i = 0; i < x.length; i++) { s += x[i]; out[i] = s; }
        return out;
    }

    private static void reverseInPlace(double[] a) {
        for (int i=0, j=a.length-1; i<j; i++, j--) {
            double tmp = a[i]; a[i] = a[j]; a[j] = tmp;
        }
    }

    private static double interp1(double x, double[] xp, double[] fp) {
        if (xp == null || fp == null || xp.length == 0) return 0.0;
        if (xp.length == 1) return fp[0];
        if (x <= xp[0]) return fp[0];
        final int n = xp.length;
        if (x >= xp[n-1]) return fp[n-1];

        int lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (x >= xp[mid]) lo = mid; else hi = mid;
        }
        final double x0 = xp[lo], x1 = xp[hi];
        final double y0 = fp[lo], y1 = fp[hi];
        final double t  = (x - x0) / Math.max(1e-12, (x1 - x0));
        return y0 + t * (y1 - y0);
    }
}
