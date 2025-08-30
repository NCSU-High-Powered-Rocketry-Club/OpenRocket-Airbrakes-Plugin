package com.airbrakesplugin.util;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Apogee predictor (Python parity)
 *
 * Model: a(t) = A * (1 - B t)^4 fitted to COAST acceleration samples (accel includes gravity).
 * LUT build: integrate (model - g) -> v, then v -> H; store ΔH(apogee - H) vs ascending v.
 * Query: apogee ≈ currentAltitude + interp(ΔH at current ascending velocity).
 *
 * Feed ONLY during coast via {@link #update(double, double, double, double)}.
 * Gravity is NOT subtracted on input; it is subtracted when building the LUT.
 */
public final class ApogeePredictor {
    private static final Logger log = LoggerFactory.getLogger(ApogeePredictor.class);

    // -------------------- Configuration --------------------
    private final int    APOGEE_PREDICTION_MIN_PACKETS;  // minimum samples before first fit
    private final double FLIGHT_LENGTH_SECONDS;          // horizon for LUT integration
    private final double INTEGRATION_DT_SECONDS;         // LUT integration dt
    private final double GRAVITY_MPS2;                   // +Z up -> gravity is negative in world Z accel
    private final double UNCERTAINTY_THRESHOLD;          // convergence gate (σA, σB both below)
    private final double INIT_A;                         // initial A for fitter
    private final double INIT_B;                         // initial B for fitter
    private final int    MAX_ITERS = 60;
    private final double LM_LAMBDA = 1e-3;               // light damping

    // -------------------- State (inputs) -------------------
    private final Deque<Double> accelerations = new ArrayDeque<>(); // includes gravity
    private final Deque<Double> dts           = new ArrayDeque<>();
    private double[] cumulativeTime = new double[0];

    private double currentAltitude = 0.0;
    private double currentVelocity = 0.0;
    private Double initialVelocity = null;

    // -------------------- State (fit & LUT) ----------------
    private boolean hasConverged = false;
    private int     lastRunLength = 0;
    private double  A = -9.0, B = 0.05;
    private double[] uncertainties = new double[] { Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY };

    private double[] lutVelocities   = null;  // ascending v (m/s)
    private double[] lutDeltaHeights = null;  // ΔH to apogee (m)

    public ApogeePredictor() {
        this(
                /*minPackets*/   10,
                /*flightLen*/    120,
                /*dt*/           0.01,
                /*g*/            9.80665,
                /*uncThr*/       0.20,   // realistic for dv/dt-derived accel; tune 0.10–0.30 as needed
                /*A0*/          -10.0,
                /*B0*/           0.05
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
        this.FLIGHT_LENGTH_SECONDS         = Math.max(128, flightLengthSeconds);
        this.INTEGRATION_DT_SECONDS        = Math.max(1e-4, integrationDtSeconds);
        this.GRAVITY_MPS2                  = gravity;
        this.UNCERTAINTY_THRESHOLD         = Math.max(1e-6, uncertaintyThreshold);
        this.INIT_A                        = (initA > 0) ? -Math.abs(initA) : initA;
        this.INIT_B                        = Math.max(0.0, initB);
    }

    /** Feed coast samples: world-Z acceleration INCLUDING g, step dt, altitude, and world-Z velocity. */
    public void update(double verticalAcceleration_includingG, double dt,
                       double altitude, double verticalVelocity) {
        currentAltitude = altitude;
        currentVelocity = verticalVelocity;

        accelerations.addLast(verticalAcceleration_includingG);
        dts.addLast(Math.max(1e-6, dt));

        int newN = accelerations.size();
        double t = 0;
        cumulativeTime = new double[newN];
        int i = 0;
        for (double dti : dts) {
            t += dti;
            cumulativeTime[i++] = t;
        }

        if (newN >= APOGEE_PREDICTION_MIN_PACKETS && newN != lastRunLength) {
            // Better initial A: use first sample (should be negative)
            Double first = accelerations.peekFirst();
            double A0 = (first != null && Double.isFinite(first)) ? first : INIT_A;

            CurveCoefficients cc = curveFit(cumulativeTime, toArray(accelerations), A0, INIT_B);
            this.A = cc.A;
            this.B = cc.B;
            this.uncertainties = cc.uncertainties;

            // Debug fit (lets you tune threshold with real data)
            final int N = accelerations.size();
            final double T = (cumulativeTime.length > 0) ? cumulativeTime[cumulativeTime.length - 1] : 0.0;
            log.debug("[ApoPred] fit: A={}  B={}  σA={} σB={}  N={}  T={}s", A, B, uncertainties[0], uncertainties[1], N, T);

            if (uncertainties[0] < UNCERTAINTY_THRESHOLD && uncertainties[1] < UNCERTAINTY_THRESHOLD) {
                hasConverged = true;
            }

            // Build/refresh the LUT every time (Python does this each fit pass too)
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

    /** Best-effort output: use current LUT even if not converged (returns null if LUT trivial). */
    public Double getApogeeBestEffort() {
        if (!hasUsableLut()) return null;
        final double dH = interp1(currentVelocity, lutVelocities, lutDeltaHeights);
        return currentAltitude + dH;
    }

    public boolean hasConverged()             { return hasConverged; }
    public boolean hasUsableLut()             { return lutVelocities != null && lutVelocities.length > 2; }
    public int     sampleCount()              { return accelerations.size(); }
    public double  getA()                     { return A; }
    public double  getB()                     { return B; }
    public double[] getUncertaintiesCopy()    { return uncertainties.clone(); }
    public double getUncertaintyThreshold()   { return UNCERTAINTY_THRESHOLD; }

    private static double[] toArray(Deque<Double> q) {
        double[] out = new double[q.size()];
        int i = 0;
        for (double v : q) out[i++] = v;
        return out;
    }

    private static final class CurveCoefficients {
        final double A, B;
        final double[] uncertainties;
        CurveCoefficients(double A, double B, double[] uncertainties) {
            this.A = A; this.B = B; this.uncertainties = uncertainties;
        }
    }

    /** Nonlinear LS (Gauss–Newton with light LM damping) for a(t) = A(1 - B t)^4.
     *  Enforces physical constraints: A <= 0 (downward accel in +Z), B >= 0.
     */
    private CurveCoefficients curveFit(double[] t, double[] a, double A0, double B0) {
        double A = (A0 > 0) ? -Math.abs(A0) : A0;
        double B = Math.max(0.0, B0);

        for (int iter = 0; iter < MAX_ITERS; iter++) {
            double H11 = 0, H12 = 0, H22 = 0;
            double g1 = 0, g2 = 0;
            double rss = 0;

            for (int i = 0; i < t.length; i++) {
                final double ti = t[i];
                final double oneMinusBt = 1.0 - B * ti;
                final double oneMinusBt2 = oneMinusBt * oneMinusBt;
                final double oneMinusBt3 = oneMinusBt2 * oneMinusBt;
                final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

                // model and residual
                final double fi = A * oneMinusBt4;
                final double ri = a[i] - fi;
                rss += ri * ri;

                // Jacobian entries
                final double dfdA = oneMinusBt4;
                final double dfdB = -4.0 * A * ti * oneMinusBt3;

                // Accumulate normal equations with small LM damping
                H11 += dfdA * dfdA;
                H12 += dfdA * dfdB;
                H22 += dfdB * dfdB;
                g1  += dfdA * ri;
                g2  += dfdB * ri;
            }

            // Levenberg–Marquardt damping
            final double lam = LM_LAMBDA;
            H11 += lam;
            H22 += lam;

            final double det = H11 * H22 - H12 * H12;
            if (Math.abs(det) < 1e-18) break;

            // Solve for parameter increment [dA, dB]^T
            final double dA = ( H22 * g1 - H12 * g2) / det;
            final double dB = (-H12 * g1 + H11 * g2) / det;

            // Update with mild step limiting to avoid overshoot
            A += Math.max(-10.0, Math.min(10.0, dA));
            B += Math.max(-1.0,  Math.min(1.0,  dB));

            // Enforce physical constraints each step
            if (A > 0) A = -Math.abs(A);
            if (B < 0) B = 0.0;

            // Simple stagnation test
            if (Math.abs(dA) < 1e-9 && Math.abs(dB) < 1e-9) break;
        }

        // Uncertainty (approximate): from inverse JTJ scaled by residual variance
        double H11 = 0, H12 = 0, H22 = 0;
        double rss = 0;
        for (int i = 0; i < t.length; i++) {
            final double ti = t[i];
            final double oneMinusBt = 1.0 - B * ti;
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
        return new CurveCoefficients(A, B, new double[]{sA, sB});
    }

    
    /** Build ΔH(v) LUT starting from the **current** state.
     *  - Integrate model (world-Z accel INCLUDING g) → v
     *  - Integrate v → altitude
     *  - Up to apogee index, collect pairs (v, ΔH = H_apogee − H(t))
     *  - Sort x-axis ascending in v (0 … v_current) for interpolation.
     */
    private void updatePredictionLookupTable(double A, double B) {
        // Time grid for coast prediction
        final int N = Math.max(2, (int) Math.ceil(FLIGHT_LENGTH_SECONDS / INTEGRATION_DT_SECONDS));
        double[] acc  = new double[N];
        double[] vel  = new double[N];
        double[] alt  = new double[N];

        // Model accel (already includes g); clamp (1 - B t) to avoid extreme powers
        for (int i = 0; i < N; i++) {
            final double ti = i * INTEGRATION_DT_SECONDS;
            double oneMinusBt = 1.0 - B * ti;
            if (oneMinusBt >  5.0) oneMinusBt =  5.0;
            if (oneMinusBt < -5.0) oneMinusBt = -5.0;
            final double oneMinusBt2 = oneMinusBt * oneMinusBt;
            final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

            final double modelAccel = A * oneMinusBt4;   // world-Z accel, includes g
            acc[i] = modelAccel;
        }

        // Integrate from the **current** state
        double v = currentVelocity;
        double h = currentAltitude;
        double hMax = h;
        int iAp = 0;

        for (int i = 0; i < N; i++) {
            v += acc[i] * INTEGRATION_DT_SECONDS;
            vel[i] = v;

            h += v * INTEGRATION_DT_SECONDS;
            alt[i] = h;

            if (h > hMax) { hMax = h; iAp = i; }
            // Optional early stop if we've clearly passed apogee for a while:
            if (i > 5 && vel[i] <= 0.0 && alt[i] < alt[i-1] && iAp > 0) break;
        }

        // Build ΔH(v) up to apogee index, with x-axis **ascending** in v (0 … v_current).
        final int M = Math.max(2, iAp + 1);
        double[] vAsc  = new double[M];
        double[] dHAsc = new double[M];

        // Reverse so that v goes from ~0 up to current v (ascending)
        for (int k = 0; k < M; k++) {
            final int i = M - 1 - k;
            vAsc[k]  = Math.max(0.0, vel[i]);
            dHAsc[k] = hMax - alt[i];
        }

        // Ensure strictly increasing x-axis for robust interp
        for (int k = 1; k < M; k++) {
            if (vAsc[k] <= vAsc[k - 1]) vAsc[k] = vAsc[k - 1] + 1e-9;
        }

        this.lutVelocities   = vAsc;
        this.lutDeltaHeights = dHAsc;
    }

    /** Linear 1D interpolation with boundary clamp. */
    private static double interp1(double x, double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length == 0) return 0.0;
        if (x <= xs[0]) return ys[0];
        int n = xs.length;
        if (x >= xs[n - 1]) return ys[n - 1];

        int lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (xs[mid] <= x) lo = mid; else hi = mid;
        }
        double x0 = xs[lo], x1 = xs[hi];
        double y0 = ys[lo], y1 = ys[hi];
        double t = (x - x0) / Math.max(1e-12, (x1 - x0));
        return y0 + t * (y1 - y0);
    }
}
