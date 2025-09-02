package com.airbrakesplugin.util;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

/**
 * Lightweight, self-contained apogee predictor.
 *
 * Model (coast only): a(t) = A * (1 - B t)^4   where a is world-Z acceleration INCLUDING g.
 *
 * How it works
 * 1) As samples arrive, fit A,B to the recent acceleration-vs-time using Gauss–Newton with light
 *    Levenberg–Marquardt damping. Enforce A <= 0 (downward accel in +Z), B >= 0.
 * 2) From the CURRENT state (altitude, vertical velocity), integrate the fitted model forward in time:
 *       v_{i+1} = v_i + a_model(t_i) * dt
 *       h_{i+1} = h_i + v_{i+1} * dt
 *    until apogee (max(h)) is reached.
 * 3) Build a lookup table of ΔH = H_apogee − H(t) against the ascending velocities v(t) in [0, v_current].
 *    Apogee estimate = currentAltitude + ΔH at currentVelocity (1D linear interpolation).
 *
 * Two outputs:
 *  - {@link #getApogeeBestEffort()}  : available as soon as a LUT can be built (>=2 points).
 *  - {@link #getPredictionIfReady()} : same, but only when the fit uncertainties are small enough.
 */
public final class ApogeePredictor {
    private static final Logger log = LoggerFactory.getLogger(ApogeePredictor.class);

    // ---------------- Configuration ----------------
    private final int    APOGEE_PREDICTION_MIN_PACKETS;
    private final double FLIGHT_LENGTH_SECONDS;
    private final double INTEGRATION_DT_SECONDS;
    private final double GRAVITY_MPS2;
    private final double UNCERTAINTY_THRESHOLD;
    private final double INIT_A;
    private final double INIT_B;
    private final int    MAX_ITERS = 60;
    private final double LM_LAMBDA = 1e-3;

    // ---------------- State (inputs) ---------------
    private final Deque<Double> accelerations = new ArrayDeque<>(); // includes gravity
    private final Deque<Double> dts           = new ArrayDeque<>();
    private double[] cumulativeTime = new double[0];

    private double currentAltitude = 0.0;
    private double currentVelocity = 0.0;
    private double tFitNow = 0.0;

    // ---------------- State (fit & LUT) ------------
    private boolean hasConverged = false;
    private int     lastRunLength = 0;
    private double  A = -9.0, B = 0.05;
    private double[] uncertainties = new double[] { Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY };

    private double[] lutVelocities   = null;  // ascending v (m/s)
    private double[] lutDeltaHeights = null;  // ΔH to apogee (m)

    // ------------- Constructors --------------------
    public ApogeePredictor() {
        this(
            /*minPackets*/   10,
            /*flightLen*/    120.0,
            /*dt*/           0.01,
            /*g*/            9.80665,
            /*uncert*/       0.25,
            /*initA*/        -9.0,
            /*initB*/        0.05
        );
    }

    public ApogeePredictor(int minPackets,
                           double flightLengthSeconds,
                           double integrationDtSeconds,
                           double gravity,
                           double uncertaintyThreshold,
                           double initA,
                           double initB) {
        this.APOGEE_PREDICTION_MIN_PACKETS = Math.max(2, minPackets);
        this.FLIGHT_LENGTH_SECONDS         = Math.max(5.0, flightLengthSeconds);
        this.INTEGRATION_DT_SECONDS        = Math.max(1e-4, integrationDtSeconds);
        this.GRAVITY_MPS2                  = gravity;
        this.UNCERTAINTY_THRESHOLD         = Math.max(1e-8, uncertaintyThreshold);
        this.INIT_A                        = (initA > 0) ? -Math.abs(initA) : initA;
        this.INIT_B                        = Math.max(0.0, initB);
    }

    // ------------- Public API ----------------------

    /** Feed **coast** samples: world-Z acceleration INCLUDING g, time step (s), altitude (m), vertical velocity (m/s). */
    public void update(double verticalAcceleration_includingG, double dt, double altitude, double verticalVelocity) {
        currentAltitude = altitude;
        currentVelocity = verticalVelocity;

        accelerations.addLast(verticalAcceleration_includingG);
        dts.addLast(Math.max(1e-6, dt));

        // Recompute cumulative time
        final int newN = accelerations.size();
        cumulativeTime = new double[newN];

        double tCum = 0.0;
        int idx = 0;
        for (double di : dts) {
            cumulativeTime[idx++] = tCum;  // timestamp for this sample BEFORE adding its dt
            tCum += di;
        }

        // Track the time of the latest sample (used as t0 when forecasting)
        if (newN > 0) {
            tFitNow = cumulativeTime[newN - 1];
        }

        if (newN >= APOGEE_PREDICTION_MIN_PACKETS && newN != lastRunLength) {
            // Initial A guess = first sample (should be near -g), B guess = INIT_B
            Double first = accelerations.peekFirst();
            double A0 = (first != null && Double.isFinite(first)) ? first : INIT_A;

            CurveCoefficients cc = curveFit(cumulativeTime, toArray(accelerations), A0, INIT_B);
            this.A = cc.A;
            this.B = cc.B;
            this.uncertainties = cc.uncertainties;

            if (log.isDebugEnabled()) {
                final double T = cumulativeTime.length > 0 ? cumulativeTime[cumulativeTime.length - 1] : 0.0;
                log.debug("[ApoPred] fit: A={}  B={}  σA={}  σB={}  N={}  T={}s", A, B, uncertainties[0], uncertainties[1], newN, T);
            }

            if (uncertainties[0] < UNCERTAINTY_THRESHOLD && uncertainties[1] < UNCERTAINTY_THRESHOLD) {
                hasConverged = true;
            }

            // Always refresh LUT from the CURRENT state
            updatePredictionLookupTable(this.A, this.B);

            lastRunLength = newN;
        }
    }

    /** Strict output: null until fit uncertainties are below threshold. */
    public Double getPredictionIfReady() {
        if (!hasConverged) return null;
        if (!hasUsableLut()) return null;
        final double dH = interp1(currentVelocity, lutVelocities, lutDeltaHeights);
        return currentAltitude + dH;
    }

    /** Best-effort output: uses current LUT even if not converged (null if LUT not yet built). */
    public Double getApogeeBestEffort() {
        if (!hasUsableLut()) return null;
        final double dH = interp1(currentVelocity, lutVelocities, lutDeltaHeights);
        return currentAltitude + dH;
    }

    public boolean hasConverged()             { return hasConverged; }
    public boolean hasUsableLut() { return lutVelocities != null && lutVelocities.length >= 2; }
    public int     sampleCount()              { return accelerations.size(); }
    public double  getA()                     { return A; }
    public double  getB()                     { return B; }
    public double[] getUncertainties()        { return Arrays.copyOf(uncertainties, uncertainties.length); }

    // ------------- Fitting -------------------------

    private static final class CurveCoefficients {
        final double A, B;
        final double[] uncertainties;
        CurveCoefficients(double A, double B, double[] uncertainties) {
            this.A = A; this.B = B; this.uncertainties = uncertainties;
        }
    }

    /** Nonlinear LS (Gauss–Newton + light LM) for a(t) = A (1 - B t)^4 with A<=0, B>=0. */
    private CurveCoefficients curveFit(double[] t, double[] a, double A0, double B0) {
        double A = (A0 > 0) ? -Math.abs(A0) : A0;
        double B = Math.max(0.0, B0);

        for (int iter = 0; iter < MAX_ITERS; iter++) {
            double H11 = 0, H12 = 0, H22 = 0;
            double g1  = 0, g2  = 0;
            double rss = 0;

            for (int i = 0; i < t.length; i++) {
                final double ti = t[i];
                final double oneMinusBt = clamp(1.0 - B * ti, -5.0, 5.0);
                final double oneMinusBt2 = oneMinusBt * oneMinusBt;
                final double oneMinusBt3 = oneMinusBt2 * oneMinusBt;
                final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

                final double fi = A * oneMinusBt4;
                final double ri = a[i] - fi;        // residual (data - model)
                rss += ri * ri;

                final double dfdA = oneMinusBt4;
                final double dfdB = -4.0 * A * ti * oneMinusBt3;

                H11 += dfdA * dfdA;
                H12 += dfdA * dfdB;
                H22 += dfdB * dfdB;

                g1  += dfdA * ri;   // J^T r
                g2  += dfdB * ri;
            }

            // Levenberg–Marquardt
            final double lam = LM_LAMBDA;
            H11 += lam;
            H22 += lam;

            final double det = H11 * H22 - H12 * H12;
            if (Math.abs(det) < 1e-18) break;

            final double dA = ( H22 * g1 - H12 * g2) / det;
            final double dB = (-H12 * g1 + H11 * g2) / det;

            // Conservative step limiting
            A += clamp(dA, -10.0, 10.0);
            B += clamp(dB,  -1.0,  1.0);

            // Enforce physical constraints
            if (A > 0) A = -Math.abs(A);
            if (B < 0) B = 0.0;

            // Stagnation
            if (Math.abs(dA) < 1e-9 && Math.abs(dB) < 1e-9) break;
        }

        // Approximate covariance from (J^T J)^{-1} scaled by residual variance
        double H11 = 0, H12 = 0, H22 = 0;
        double rss = 0;
        for (int i = 0; i < t.length; i++) {
            final double ti = t[i];
            final double oneMinusBt = clamp(1.0 - B * ti, -5.0, 5.0);
            final double oneMinusBt2 = oneMinusBt * oneMinusBt;
            final double oneMinusBt3 = oneMinusBt2 * oneMinusBt;
            final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

            final double fi = A * oneMinusBt4;
            final double ri = a[i] - fi;
            rss += ri * ri;

            final double dfdA = oneMinusBt4;
            final double dfdB = -4.0 * A * ti * oneMinusBt3;
            H11 += dfdA * dfdA;
            H12 += dfdA * dfdB;
            H22 += dfdB * dfdB;
        }
        final double det = H11 * H22 - H12 * H12;
        final double s2  = (t.length > 2) ? rss / (t.length - 2) : rss;

        double varA, varB;
        if (Math.abs(det) >= 1e-18) {
            varA = s2 * ( H22 / det);
            varB = s2 * ( H11 / det);
        } else {
            varA = varB = Double.POSITIVE_INFINITY;
        }
        final double sA = varA > 0 ? Math.sqrt(varA) : Double.POSITIVE_INFINITY;
        final double sB = varB > 0 ? Math.sqrt(varB) : Double.POSITIVE_INFINITY;

        return new CurveCoefficients(A, B, new double[] { sA, sB });
    }

    // ------------- LUT build -----------------------

        /**
         * Build ΔH(v) table from current state by integrating the fitted model forward until apogee.
         */
        private void updatePredictionLookupTable(double A, double B) {
        final int N = Math.max(2, (int) Math.ceil(FLIGHT_LENGTH_SECONDS / INTEGRATION_DT_SECONDS));
        final double t0 = this.tFitNow;  // <-- time offset from last sample

        double[] vel = new double[N];
        double[] alt = new double[N];

        // Integrate forward from the CURRENT state, using a(t0 + tau)
        double v    = currentVelocity;
        double h    = currentAltitude;
        double hMax = h;
        int    iAp  = 0;

        for (int i = 0; i < N; i++) {
            final double tau = i * INTEGRATION_DT_SECONDS;
            double oneMinusBt = 1.0 - B * (t0 + tau);
            // keep the gentle clamp if you had it; it does not bind near apogee for realistic t
            if (oneMinusBt >  5.0) oneMinusBt =  5.0;
            if (oneMinusBt < -5.0) oneMinusBt = -5.0;

            final double oneMinusBt2 = oneMinusBt * oneMinusBt;
            final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

            final double aModel = A * oneMinusBt4; // includes g
            v += aModel * INTEGRATION_DT_SECONDS;
            vel[i] = v;

            h += v * INTEGRATION_DT_SECONDS;
            alt[i] = h;

            if (h > hMax) { hMax = h; iAp = i; }
            if (i > 5 && vel[i] <= 0.0 && alt[i] < alt[i - 1] && iAp > 0) break;
        }

        // Use points up to and including apogee; add a terminal anchor at the current state
        final int core = Math.max(2, iAp + 1);
        final int M = core + 1;

        double[] vAsc  = new double[M];
        double[] dHAsc = new double[M];

        // Fill reversed so v is ascending from ~0 to current v
        for (int k = 0; k < core; k++) {
            final int i = core - 1 - k;
            vAsc[k]  = Math.max(0.0, vel[i]);
            dHAsc[k] = hMax - alt[i];
        }

        // Terminal anchor = exactly the current state (ensures interpolation at v_current)
        vAsc[M - 1]  = Math.max(0.0, currentVelocity);
        dHAsc[M - 1] = Math.max(0.0, hMax - currentAltitude);

        // Ensure strictly increasing x
        for (int k = 1; k < M; k++) {
            if (vAsc[k] <= vAsc[k - 1]) vAsc[k] = vAsc[k - 1] + 1e-9;
        }

        this.lutVelocities   = vAsc;
        this.lutDeltaHeights = dHAsc;
    }


    // ------------- helpers -------------------------

    private static double clamp(double x, double lo, double hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }

    private static double[] toArray(Deque<Double> dq) {
        final double[] out = new double[dq.size()];
        int i = 0;
        for (double v : dq) out[i++] = v;
        return out;
    }

    /** Linear interpolation with boundary clamp. */
    private static double interp1(double x, double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length == 0) return 0.0;
        if (x <= xs[0]) return ys[0];
        final int n = xs.length;
        if (x >= xs[n-1]) return ys[n-1];

        int lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (xs[mid] <= x) lo = mid; else hi = mid;
        }
        final double x0 = xs[lo], x1 = xs[hi];
        final double y0 = ys[lo], y1 = ys[hi];
        final double t = (x - x0) / Math.max(1e-12, (x1 - x0));
        return y0 + t * (y1 - y0);
    }
}
