package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.ExtrapolationType;
import com.airbrakesplugin.util.GenericFunction2D;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;
import java.util.Objects;

/**
 * AirbrakeAerodynamics
 *
 * Interpolates ΔDrag (Newtons) as a function of (Mach, DeploymentFraction).
 * - CSV must contain a full rectangular grid with columns:
 *     Mach, DeploymentPercentage, DeltaDrag_N
 * - Interpolation: bilinear (robust/monotone).
 * - Extrapolation: CONSTANT (edge clamp) by default.
 *
 * Notes:
 * - DeploymentFraction is expected in [0..1]. Values outside are clamped.
 * - All previous Cd/Cm bicubic logic has been removed for reliability.
 */
public final class AirbrakeAerodynamics {

    private static final Logger LOG = LoggerFactory.getLogger(AirbrakeAerodynamics.class);

    /** Tiny numeric epsilon used for domain nudging when needed. */
    private static final double EPS = 1e-12;

    /** Drag surface in Newtons (ΔDrag_N). */
    private GenericFunction2D deltaDragSurface;

    /** Path kept only for diagnostics/reload if desired. */
    private Path sourceCsv;

    /**
     * Build from a CSV file path. The path can be absolute or relative to the working dir
     * or a deployed resource path that you copy to disk before calling this.
     */
    public AirbrakeAerodynamics(String csvFilePath) {
        Objects.requireNonNull(csvFilePath, "CSV file path must not be null");
        if (csvFilePath.isBlank()) {
            throw new IllegalArgumentException("CSV file path is empty");
        }
        loadDeltaDragSurface(Path.of(csvFilePath), /*extrap*/ ExtrapolationType.CONSTANT);
    }

    /**
     * Alternate constructor if you want to pass an already-resolved Path and choose extrapolation.
     */
    public AirbrakeAerodynamics(Path csvPath, ExtrapolationType extrapolation) {
        loadDeltaDragSurface(csvPath, extrapolation == null ? ExtrapolationType.CONSTANT : extrapolation);
    }

    /**
     * (Re)load the ΔDrag surface from CSV.
     */
    public final void loadDeltaDragSurface(Path csvPath, ExtrapolationType extrapolation) {
        Objects.requireNonNull(csvPath, "CSV path must not be null");
        Objects.requireNonNull(extrapolation, "Extrapolation must not be null");
        try {
            this.deltaDragSurface = GenericFunction2D.fromCsv(csvPath, extrapolation);
            this.sourceCsv = csvPath;
            LOG.info("Loaded airbrake ΔDrag surface: {} (extrapolation: {})", csvPath, extrapolation);
        } catch (Exception e) {
            LOG.error("Failed to load ΔDrag surface from {}: {}", csvPath, e.toString());
            throw (e instanceof IllegalArgumentException) ? (IllegalArgumentException) e
                    : new IllegalArgumentException("Cannot load ΔDrag surface: " + csvPath, e);
        }
    }

    // -----------------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------------

    /**
     * Interpolate ΔDrag (Newtons) at the given Mach and deployment fraction.
     *
     * @param mach         Freestream Mach number (finite; will be clamped at grid edges).
     * @param deployFrac   Deployment fraction in [0..1] (will be clamped).
     * @return ΔDrag in Newtons (>= 0 typically), or 0.0 if the surface is not loaded.
     */
    public double getDeltaDragN(double mach, double deployFrac) {
        if (deltaDragSurface == null) {
            LOG.warn("ΔDrag surface not loaded; returning 0 N");
            return 0.0;
        }
        if (!Double.isFinite(mach)) {
            LOG.trace("Non-finite Mach={} → using 0.0", mach);
            mach = 0.0;
        }
        if (!Double.isFinite(deployFrac)) {
            LOG.trace("Non-finite deploy={} → using 0.0", deployFrac);
            deployFrac = 0.0;
        }
        // Clamp deployment to [0..1] and nudge slightly to avoid exact boundaries if needed.
        double dep = clamp01(deployFrac);
        dep = nudge(dep, 0.0, 1.0);

        try {
            double val = deltaDragSurface.value(mach, dep);
            if (Double.isFinite(val)) return val;
        } catch (Exception ex) {
            LOG.debug("ΔDrag interpolation failed at Mach={}, Deploy={} from {}: {}",
                    mach, dep, sourceCsv, ex.toString());
        }
        return 0.0;
    }

    /**
     * Convenience wrapper used by the SimulationListener:
     * computes Mach from (speed, altitudeMSL) and returns ΔDrag [N] for the given deployment.
     *
     * @param deploymentFrac  deployment fraction [0..1]
     * @param speed           speed magnitude [m/s]
     * @param altitudeMSL     altitude above mean sea level [m]
     * @return ΔDrag in Newtons
     */
    public double calculateDragForce(double deploymentFrac, double speed, double altitudeMSL) {
        if (!isReady()) return 0.0;
        if (!Double.isFinite(speed) || speed <= 0) return 0.0;
        if (!Double.isFinite(altitudeMSL)) altitudeMSL = 0.0;

        final double mach = AirDensity.machFromV(speed, altitudeMSL);
        final double dep  = clamp01(deploymentFrac);
        final double dN   = getDeltaDragN(mach, dep);

        LOG.trace("calculateDragForce: M={} dep={} → ΔDrag={} N (speed={}, altMSL={})",
                mach, dep, dN, speed, altitudeMSL);
        return dN;
    }

    /** @return true if the ΔDrag surface is loaded. */
    public boolean isReady() {
        return deltaDragSurface != null;
    }

    // -----------------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------------

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    private static double nudge(double v, double min, double max) {
        // Pull off exact boundaries by an extremely small amount when helpful.
        if (v <= min) return Math.nextUp(min + EPS);
        if (v >= max) return Math.nextDown(max - EPS);
        return v;
    }
}
