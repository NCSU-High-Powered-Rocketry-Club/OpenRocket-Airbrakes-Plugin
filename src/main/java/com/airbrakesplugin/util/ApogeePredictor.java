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

    // ---------------- Debug trace ------------------
    public interface TraceSink {
        void onUpdate(double t, double alt, double vWorldZ, double aWorldZ, double apogeeStrict, double apogeeBestEffort, double uncertainty, int packets, String note);
    }
    private TraceSink traceSink = null;
    public void setTraceSink(TraceSink sink) { this.traceSink = sink; }
    private void publishTrace(double t, double alt, double vz, double az, double apoStrict, double apoBest, double unc, int packets, String note) {
        if (traceSink != null) {
            traceSink.onUpdate(t, alt, vz, az, apoStrict, apoBest, unc, packets, note);
        }
    }

    // ------------- Constructors --------------------
    public ApogeePredictor() {
        this(
            /*minPackets*/   5,  // Lowered for faster readiness
            /*flightLen*/    120.0,
            /*dt*/           0.01,
            /*g*/            9.80665,
            /*uncert*/       0.25,
            /*initA*/        -9.0,
            /*initB*/        0.05
        );
    }

    public ApogeePredictor(int minPackets, double flightLengthSeconds, double integrationDtSeconds, double gravity, double uncertaintyThreshold, double initA, double initB) {
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
        // Skip if accel positive (safety for non-coast phases)
        if (verticalAcceleration_includingG > 0.0) {
            log.warn("[ApoPred] skipped update: positive accel {} (non-coast?)", verticalAcceleration_includingG);
            return;
        }

        currentAltitude = altitude;
        currentVelocity = verticalVelocity;

        accelerations.addLast(verticalAcceleration_includingG);
        dts.addLast(Math.max(1e-6, dt));

        // Recompute cumulative time
        final int n = dts.size();
        
        if (cumulativeTime.length < n) cumulativeTime = new double[n];
        double cumT = 0.0;
        int i = 0;
        
        for (double dti : dts) {
            cumulativeTime[i++] = cumT;
            cumT += dti;
        }
        tFitNow = cumT;

        // Trim to recent history if too long (optional cap, e.g., last 1000)
        while (accelerations.size() > 1000) {
            accelerations.removeFirst();
            dts.removeFirst();
        }

        // Fit if enough points
        final int len = accelerations.size();
        if (len >= APOGEE_PREDICTION_MIN_PACKETS && len != lastRunLength) {
            final double[] a = toArray(accelerations);
            final double[] t = Arrays.copyOf(cumulativeTime, len);
            final CurveCoefficients coeffs = fitCurve(a, t);
            if (coeffs != null) {
                A = coeffs.A; B = coeffs.B; uncertainties = coeffs.uncertainties;
                hasConverged = true;
                log.debug("[ApoPred] fit: A={} B={} σA={} σB={} N={} T={}s", fmt(A), fmt(B), fmt(uncertainties[0]), fmt(uncertainties[1]), len, fmt(tFitNow));

                // Build LUT for predictions
                updatePredictionLookupTable(A, B);

                // Publish trace (if sink set)
                if (traceSink != null) {
                    double unc = Math.max(uncertainties[0] / Math.abs(A), uncertainties[1] / Math.max(1e-6, B));
                    Double apoStrictObj = getPredictionIfReady();
                    double apoStrict = (apoStrictObj != null) ? apoStrictObj : Double.NaN;
                    Double apoBestObj = getApogeeBestEffort();
                    double apoBest = (apoBestObj != null) ? apoBestObj : Double.NaN;
                    publishTrace(tFitNow, currentAltitude, currentVelocity, verticalAcceleration_includingG, apoStrict, apoBest, unc, len, "fit complete");
                }
            }
            lastRunLength = len;
        }
    }

    public void reset() {
        accelerations.clear();
        dts.clear();
        cumulativeTime = new double[0];
        tFitNow = 0.0;
        hasConverged = false;
        lastRunLength = 0;
        A = INIT_A; B = INIT_B;
        uncertainties = new double[] { Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY };
        lutVelocities = null;
        lutDeltaHeights = null;
    }

    public Double getPredictionIfReady() {
        if (!hasConverged || uncertainties[0] > UNCERTAINTY_THRESHOLD * Math.abs(A) ||
            uncertainties[1] > UNCERTAINTY_THRESHOLD * Math.max(1e-6, B)) {
            return null;
        }
        return getApogeeBestEffort();
    }

    public Double getApogeeBestEffort() {
        if (lutVelocities == null || lutDeltaHeights == null || currentVelocity <= 0.0) return null;
        final double deltaH = interp1(currentVelocity, lutVelocities, lutDeltaHeights);
        return currentAltitude + deltaH;
    }

    // ------------- Fit logic -----------------------

    private static final class CurveCoefficients {
        final double A, B;
        final double[] uncertainties;
        CurveCoefficients(double A, double B, double[] unc) { this.A = A; this.B = B; this.uncertainties = unc; }
    }

    private CurveCoefficients fitCurve(double[] a, double[] t) {
        final int n = a.length;
        if (n < 2) return null;

        // Gauss-Newton with LM damping
        double A_est = INIT_A;
        double B_est = INIT_B;

        for (int iter = 0; iter < MAX_ITERS; iter++) {
            double J11 = 0, J12 = 0, J21 = 0, J22 = 0;
            double r1 = 0, r2 = 0;

            for (int i = 0; i < n; i++) {
                final double ti = t[i];
                final double oneMinusBt = clamp(1.0 - B_est * ti, -5.0, 5.0);
                final double oneMinusBt2 = oneMinusBt * oneMinusBt;
                final double oneMinusBt3 = oneMinusBt2 * oneMinusBt;
                final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

                final double f = A_est * oneMinusBt4;
                final double r = a[i] - f;

                final double dfdA = oneMinusBt4;
                final double dfdB = -4.0 * A_est * ti * oneMinusBt3;

                J11 += dfdA * dfdA;
                J12 += dfdA * dfdB;
                J21 += dfdB * dfdA;
                J22 += dfdB * dfdB;

                r1 += r * dfdA;
                r2 += r * dfdB;
            }

            // LM damping
            J11 += LM_LAMBDA * J11;
            J22 += LM_LAMBDA * J22;

            final double det = J11 * J22 - J12 * J21;
            if (Math.abs(det) < 1e-18) break;

            final double dA = (J22 * r1 - J12 * r2) / det;
            final double dB = (J11 * r2 - J21 * r1) / det;

            A_est += dA;
            B_est += dB;

            // Enforce constraints
            if (A_est > 0.0) A_est = -Math.abs(A_est);  // A <= 0
            B_est = Math.max(0.0, B_est);

            if (Math.abs(dA) < 1e-6 && Math.abs(dB) < 1e-6) break;
        }

        // Approximate covariance from (J^T J)^{-1} scaled by residual variance
        double H11 = 0, H12 = 0, H22 = 0;
        double rss = 0;
        
        for (int i = 0; i < t.length; i++) {
            final double ti = t[i];
            final double oneMinusBt = clamp(1.0 - B_est * ti, -5.0, 5.0);
            final double oneMinusBt2 = oneMinusBt * oneMinusBt;
            final double oneMinusBt3 = oneMinusBt2 * oneMinusBt;
            final double oneMinusBt4 = oneMinusBt2 * oneMinusBt2;

            final double fi = A_est * oneMinusBt4;
            final double ri = a[i] - fi;
            rss += ri * ri;

            final double dfdA = oneMinusBt4;
            final double dfdB = -4.0 * A_est * ti * oneMinusBt3;
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

        return new CurveCoefficients(A_est, B_est, new double[] { sA, sB });
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

    private static String fmt(double x) { return String.format("%.3f", x); }
}