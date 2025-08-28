package com.airbrakesplugin.util;

import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Robust apogee predictor for OpenRocket 24.12.
 *
 * - Uses OR sim data (velocity, position, orientation) directly.
 * - Projects onto rocket body Z-axis and subtracts gravity along that axis.
 * - Latches "coast" when axis accel (minus g) becomes small/negative.
 * - Builds velocity→Δheight lookup continuously (no convergence gate).
 * - Exposes a non-zero apogee ASAP using a ballistic fallback until LUT is ready.
 */
public final class ApogeePredictorListener extends AbstractSimulationListener {

    // ----- Tunables (match your Python as needed) -----
    private static final double G = 9.80665;          // m/s^2
    private static final double INTEGRATION_DT = 0.01;
    private static final double HORIZON_S      = 120.0;
    private static final int    MIN_NEW_SAMPLES_TO_REFIT = 8;
    private static final double UNCERTAINTY_THRESHOLD = 1e-2;

    // Coast detection
    private static final double COAST_ACCEL_THRESHOLD = 1.5; // m/s^2 (a_minus_g <= +1.5 ≈ burnout)
    private static final int    COAST_CONSECUTIVE     = 2;   // need N consecutive samples
    private static final double FALLBACK_TIMEOUT_S    = 1.0; // latch even if not detected after this

    // ----- State -----
    private final Deque<Double> aMinusG = new ArrayDeque<>();
    private final Deque<Double> dts     = new ArrayDeque<>();

    private boolean coastLatched = false;
    private int coastHitCount = 0;
    private double firstSampleTime = Double.NaN;

    private double lastTime = Double.NaN;
    private Double lastVaxis = null;

    private double currentAlt = 0.0;
    private double currentVaxis = 0.0;
    private Double v0 = null;

    private boolean hasConverged = false;
    private int lastRefitCount = 0;

    // Fit params & pseudo-uncertainties
    private double A = 0.0, B = 0.0;
    private double sigmaA = Double.POSITIVE_INFINITY, sigmaB = Double.POSITIVE_INFINITY;

    // LUT (ascending)
    private double[] lutV = new double[]{0.0, 0.1};
    private double[] lutDH = new double[]{0.1, 0.1};

    private double latestApogee = Double.NaN;

    // ----- Public API -----
    public boolean hasConverged() { return hasConverged; }
    public double getLatestPredictedApogee() { return latestApogee; }
    public double getFitA() { return A; }
    public double getFitB() { return B; }

    @Override
    public void postStep(final SimulationStatus status) {
        // 1) Gather live data
        final double t = status.getSimulationTime();

        final Coordinate vWorld = status.getRocketVelocity(); // m/s
        final Quaternion q = status.getRocketOrientationQuaternion();
        // Transform world vectors into rocket frame via inverse rotation
        final Coordinate vBody = q.invRotate(vWorld);
        final double vAxis = vBody.z;    // rocket +Z (nose)
        currentVaxis = vAxis;

        final Coordinate pos = status.getRocketPosition();
        currentAlt = pos.z;

        // Axis gravity component in rocket frame
        final Coordinate gWorld = new Coordinate(0, 0, -G);
        final double gAxis = q.invRotate(gWorld).z;

        // 2) Build samples (after boost)
        if (!Double.isNaN(lastTime) && lastVaxis != null) {
            final double dt = Math.max(1e-6, t - lastTime);
            final double aNetAxis = (vAxis - lastVaxis) / dt; // includes gravity & aero
            final double aMinusGaxis = aNetAxis - gAxis;

            if (Double.isNaN(firstSampleTime)) firstSampleTime = t;

            // Coast latch logic
            if (!coastLatched) {
                if (vAxis >= 0.0 && aMinusGaxis <= COAST_ACCEL_THRESHOLD) {
                    if (++coastHitCount >= COAST_CONSECUTIVE) {
                        coastLatched = true;
                        v0 = vAxis;
                    }
                } else {
                    coastHitCount = 0;
                }
                if (!coastLatched && (t - firstSampleTime) >= FALLBACK_TIMEOUT_S) {
                    coastLatched = true;
                    v0 = vAxis;
                }
            }

            // Once latched, collect samples every step
            if (coastLatched && vAxis >= 0.0) {
                aMinusG.addLast(aMinusGaxis);
                dts.addLast(dt);
                if (v0 == null) v0 = vAxis;
            }
        }

        lastTime = t;
        lastVaxis = vAxis;

        // 3) Fit & LUT update when enough new samples exist
        if (aMinusG.size() - lastRefitCount >= MIN_NEW_SAMPLES_TO_REFIT) {
            final double[] times = cumsum(toArray(dts));
            final double[] acc   = toArray(aMinusG);

            // fit y = A * (1 - B t)^4  to accel-minus-g
            final double[] params = fitAB(times, acc, (A==0?1.0:A), (B==0?0.05:B));
            A = params[0]; B = params[1];

            final double[][] cov = covariance(times, acc, A, B);
            sigmaA = Math.sqrt(Math.max(0.0, cov[0][0]));
            sigmaB = Math.sqrt(Math.max(0.0, cov[1][1]));
            hasConverged = (sigmaA < UNCERTAINTY_THRESHOLD && sigmaB < UNCERTAINTY_THRESHOLD);

            updateLookupTable(A, B);
            lastRefitCount = aMinusG.size();
        }

        // 4) Predict apogee as soon as LUT is ready; else use ballistic fallback
        if (lutV != null && lutV.length >= 2) {
            final double dv = clamp(vAxis, lutV[0], lutV[lutV.length-1]);
            final double dH = interp1(lutV, lutDH, dv);
            latestApogee = currentAlt + dH;
        } else {
            // Ballistic fallback (dragless upper bound)
            final double vup = Math.max(0.0, vAxis);
            latestApogee = currentAlt + (vup * vup) / (2.0 * G);
        }
    }

    // ----- LUT exactly like Python (accel-minus-g -> vel -> alt -> ΔH, flipped to ascending V) -----
    private void updateLookupTable(final double a, final double b) {
        final double v0loc = (v0 != null ? v0 : currentVaxis);

        final int steps = Math.max(2, (int)Math.ceil(HORIZON_S / INTEGRATION_DT));
        final double dt = INTEGRATION_DT;

        final double[] v = new double[steps];
        double vv = v0loc;
        for (int i=0;i<steps;i++) {
            final double ti = i * dt;
            final double ai = a * Math.pow(1.0 - b * ti, 4.0); // accel-minus-g model
            vv += ai * dt;
            v[i] = vv;
        }

        int lastNonNeg = -1;
        for (int i=0;i<steps;i++) if (v[i] >= 0.0) lastNonNeg = i;
        if (lastNonNeg < 1) {
            lutV  = new double[]{0.0, 0.1};
            lutDH = new double[]{0.1, 0.1};
            return;
        }
        final int n = lastNonNeg + 1;
        final double[] vKeep = new double[n];
        System.arraycopy(v, 0, vKeep, 0, n);

        // integrate velocity to relative altitude
        final double[] h = new double[n];
        double hh = 0.0;
        for (int i=0;i<n;i++) {
            hh += vKeep[i] * dt;
            h[i] = hh;
        }
        final double hAp = h[n-1];

        // flip to ascending
        reverse(vKeep);
        reverse(h);

        final double[] dH = new double[n];
        for (int i=0;i<n;i++) dH[i] = Math.max(0.0, hAp - h[i]);

        lutV = vKeep;
        lutDH = dH;
    }

    // ----- Tiny LM fit for two parameters (Gauss–Newton with damping) -----
    private static double[] fitAB(double[] t, double[] y, double A0, double B0) {
        double A = A0, B = B0;
        final int maxIter = 2000;
        double lambda = 1e-3;

        for (int it=0; it<maxIter; it++) {
            double j11=0,j12=0,j22=0,r1=0,r2=0,rss=0;

            for (int i=0;i<t.length;i++) {
                final double ti = t[i];
                final double term = 1.0 - B*ti;
                final double term3 = term*term*term;
                final double f = A * term3 * term;  // A*(1 - B t)^4
                final double ri = y[i] - f;

                final double dA = term*term*term*term;
                final double dB = A * 4.0 * term3 * (-ti);

                j11 += dA*dA;
                j12 += dA*dB;
                j22 += dB*dB;
                r1  += dA*ri;
                r2  += dB*ri;
                rss += ri*ri;
            }

            final double d = (j11+lambda)*(j22+lambda) - j12*j12;
            if (Math.abs(d) < 1e-16) break;

            final double dAstep = ((j22+lambda)*r1 - j12*r2) / d;
            final double dBstep = (-(j12)*r1 + (j11+lambda)*r2) / d;

            final double Anew = A + dAstep;
            final double Bnew = B + dBstep;

            double rssNew = 0.0;
            for (int i=0;i<t.length;i++) {
                final double ti = t[i];
                final double f = Anew * Math.pow(1.0 - Bnew*ti, 4.0);
                final double ri = y[i] - f;
                rssNew += ri*ri;
            }

            if (rssNew < rss) { A = Anew; B = Bnew; lambda = Math.max(1e-12, lambda*0.33); }
            else { lambda = Math.min(1e6, lambda*3.0); }

            if (Math.abs(dAstep)+Math.abs(dBstep) < 1e-9) break;
        }
        return new double[]{A,B};
    }

    private static double[][] covariance(double[] t, double[] y, double A, double B) {
        double j11=0,j12=0,j22=0,rss=0;
        for (int i=0;i<t.length;i++) {
            final double ti = t[i];
            final double term = 1.0 - B*ti;
            final double term3 = term*term*term;
            final double f = A*term3*term;
            final double ri = y[i] - f;

            final double dA = term*term*term*term;
            final double dB = A * 4.0 * term3 * (-ti);

            j11 += dA*dA;
            j12 += dA*dB;
            j22 += dB*dB;
            rss += ri*ri;
        }
        final double sigma2 = rss / Math.max(1, y.length - 2);
        final double det = j11*j22 - j12*j12;
        if (Math.abs(det) < 1e-16) {
            return new double[][]{{Double.POSITIVE_INFINITY, 0},{0, Double.POSITIVE_INFINITY}};
        }
        final double inv11 =  j22 / det;
        final double inv12 = -j12 / det;
        final double inv22 =  j11 / det;
        return new double[][]{
            { sigma2 * inv11, sigma2 * inv12 },
            { sigma2 * inv12, sigma2 * inv22 }
        };
    }

    // ----- utils -----
    private static double[] toArray(Deque<Double> d) {
        final double[] out = new double[d.size()];
        int i=0; for (double v: d) out[i++] = v;
        return out;
    }
    private static double[] cumsum(double[] x) {
        final double[] out = new double[x.length];
        double s=0.0; for (int i=0;i<x.length;i++){ s+=x[i]; out[i]=s; }
        return out;
    }
    private static void reverse(double[] a) {
        for (int i=0,j=a.length-1;i<j;i++,j--){ double tmp=a[i]; a[i]=a[j]; a[j]=tmp; }
    }
    private static double interp1(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        final int n = xs.length;
        if (x >= xs[n-1]) return ys[n-1];
        int lo=0, hi=n-1;
        while (hi - lo > 1) {
            final int mid = (lo + hi) >>> 1;
            if (xs[mid] >= x) hi = mid; else lo = mid;
        }
        final double u = (x - xs[lo]) / Math.max(1e-12, (xs[hi] - xs[lo]));
        return ys[lo] + u * (ys[hi] - ys[lo]);
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
