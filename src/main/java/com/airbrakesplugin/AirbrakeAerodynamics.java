package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import com.airbrakesplugin.util.ExtrapolationType;
import com.airbrakesplugin.util.GenericFunction2D;
import com.airbrakesplugin.util.HighMachExtrapolationMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;
import java.util.Objects;

/**
 * AirbrakeAerodynamics
 *
 * Interpolates absolute airbrake Drag [N] as a function of (Mach, DeploymentFraction).
 * - CSV: flexible headers (Mach, Deployment*, Drag/Force) per GenericFunction2D.
 * - Data may be scattered; loader internally builds a rectangular grid (IDW) for bilinear interp.
 * - Extrapolation: CONSTANT (edge clamp) by default (configurable).
 *
 * Notes:
 * - DeploymentFraction is expected in [0..1]; values outside are clamped.
 * - This class returns a FORCE [N], not a coefficient. Convert to ΔCd downstream using OR's q and Aref.
 */
public final class AirbrakeAerodynamics {

    private static final Logger LOG = LoggerFactory.getLogger(AirbrakeAerodynamics.class);

    /** Tiny numeric epsilon used for domain nudging when needed. */
    private static final double EPS = 1e-12;

    /** Drag surface in Newtons (absolute airbrake drag). */
    private GenericFunction2D dragSurface;
    private HighMachExtrapolationMode highMachExtrapolationMode = HighMachExtrapolationMode.BOUNDED_NONLINEAR;
    private boolean allowNegativeDeltaDrag = false;
    private double maxMachForAerodynamicModel = 5.0;
    private double maxExtrapolatedForceMultiplier = 2.0;
    private GenericFunction2D.Evaluation lastEvaluation =
            GenericFunction2D.Evaluation.failClosed(0.0, "surface not evaluated");

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
        loadDragSurface(Path.of(csvFilePath), ExtrapolationType.CONSTANT);
    }

    public AirbrakeAerodynamics(AirbrakeConfig config) {
        Objects.requireNonNull(config, "AirbrakeConfig must not be null");
        String csvFilePath = config.getCfdDataFilePath();
        if (csvFilePath == null || csvFilePath.isBlank()) {
            throw new IllegalArgumentException("CSV file path is empty");
        }
        this.highMachExtrapolationMode = config.getHighMachExtrapolationMode();
        this.allowNegativeDeltaDrag = config.isAllowNegativeDeltaDrag();
        this.maxMachForAerodynamicModel = config.getMaxMachForAerodynamicModel();
        this.maxExtrapolatedForceMultiplier = config.getMaxExtrapolatedForceMultiplier();
        loadDragSurface(Path.of(csvFilePath), ExtrapolationType.CONSTANT);
    }

    /**
     * Alternate constructor if you want to pass an already-resolved Path and choose extrapolation.
     */
    public AirbrakeAerodynamics(Path csvPath, ExtrapolationType extrapolation) {
        loadDragSurface(csvPath, extrapolation == null ? ExtrapolationType.CONSTANT : extrapolation);
    }

    /**
     * (Re)load the Drag surface from CSV.
     */
    public final void loadDragSurface(Path csvPath, ExtrapolationType extrapolation) {
        Objects.requireNonNull(csvPath, "CSV path must not be null");
        Objects.requireNonNull(extrapolation, "Extrapolation must not be null");
        try {
            this.dragSurface = GenericFunction2D.fromCsv(csvPath, extrapolation, allowNegativeDeltaDrag);
            this.sourceCsv = csvPath;
            double maxMach = this.dragSurface.xAxis()[this.dragSurface.xAxis().length - 1];
            if (maxMach < maxMachForAerodynamicModel) {
                LOG.warn("Airbrake Drag surface max Mach {} is below configured aerodynamic model max Mach {}: {}",
                        maxMach, maxMachForAerodynamicModel, csvPath);
            }
            LOG.info("Loaded airbrake Drag surface: {} (legacy extrapolation: {}, highMachMode: {}, allowNegativeDeltaDrag: {})",
                    csvPath, extrapolation, highMachExtrapolationMode, allowNegativeDeltaDrag);
        } catch (Exception e) {
            LOG.error("Failed to load Drag surface from {}: {}", csvPath, e.toString());
            throw (e instanceof IllegalArgumentException) ? (IllegalArgumentException) e : new IllegalArgumentException("Cannot load Drag surface: " + csvPath, e);
        }
    }

    // -----------------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------------

    /**
     * Interpolate absolute airbrake Drag [N] at the given Mach and deployment fraction.
     *
     * @param mach         Freestream Mach number (finite; will be clamped at grid edges).
     * @param deployFrac   Deployment fraction in [0..1] (will be clamped).
     * @return Drag in Newtons (>= 0 typically), or 0.0 if the surface is not loaded.
     */
    public double getAirbrakeDragN(double mach, double deployFrac) {
        return Math.max(0.0, getAirbrakeDeltaDragN(mach, deployFrac));
    }

    /**
     * Interpolate or extrapolate signed airbrake delta drag [N].
     * Positive values add drag; negative values reduce baseline drag and require
     * allowNegativeDeltaDrag=true at CSV load time.
     */
    public double getAirbrakeDeltaDragN(double mach, double deployFrac) {
        if (dragSurface == null) {
            LOG.warn("Drag surface not loaded; returning 0 N");
            lastEvaluation = GenericFunction2D.Evaluation.failClosed(0.0, "surface not loaded");
            return 0.0;
        }
        if (!Double.isFinite(mach)) {
            LOG.warn("Non-finite Mach={}; failing closed", mach);
            lastEvaluation = GenericFunction2D.Evaluation.failClosed(0.0, "non-finite Mach");
            return 0.0;
        }
        if (!Double.isFinite(deployFrac)) {
            LOG.warn("Non-finite deploy={}; failing closed", deployFrac);
            lastEvaluation = GenericFunction2D.Evaluation.failClosed(0.0, "non-finite deployment");
            return 0.0;
        }
        
        // Clamp deployment to [0..1] and nudge slightly to avoid exact boundaries if needed.
        double dep = clamp01(deployFrac);
        dep = nudge(dep, 0.0, 1.0);

        try {
            GenericFunction2D.Evaluation eval = dragSurface.evaluate(
                    mach,
                    dep,
                    highMachExtrapolationMode,
                    maxMachForAerodynamicModel,
                    maxExtrapolatedForceMultiplier);
            lastEvaluation = eval;
            double valN = eval.valueN();
            if (eval.failedClosed()) {
                LOG.warn("Airbrake delta drag failed closed at Mach={}, Deploy={} from {}: {}",
                        mach, dep, sourceCsv, eval.reason());
                return 0.0;
            }
            if (Double.isFinite(valN) && valN < 0.0 && !allowNegativeDeltaDrag) {
                LOG.warn("Airbrake delta drag failed closed at Mach={}, Deploy={} from {}: negative {} N produced while allowNegativeDeltaDrag=false",
                        mach, dep, sourceCsv, valN);
                lastEvaluation = GenericFunction2D.Evaluation.failClosed(0.0, "negative delta drag rejected");
                return 0.0;
            }
            if (Double.isFinite(valN)) return valN;
        } catch (Exception ex) {
            LOG.debug("Drag interpolation failed at Mach={}, Deploy={} from {}: {}",
                    mach, dep, sourceCsv, ex.toString());
        }
        lastEvaluation = GenericFunction2D.Evaluation.failClosed(0.0, "evaluation exception");
        return 0.0;
    }

    /**
     * Convenience wrapper used by the SimulationListener:
     * computes Mach from (speed, altitudeMSL) and returns Drag [N] for the given deployment.
     *
     * @param deploymentFrac  deployment fraction [0..1]
     * @param speed           speed magnitude [m/s]
     * @param altitudeMSL     altitude above mean sea level [m]
     * @return Drag in Newtons
     */
    public double calculateDragForce(double deploymentFrac, double speed, double altitudeMSL) {
        if (!isReady()) return 0.0;
        if (!Double.isFinite(speed) || speed <= 0) return 0.0;
        if (!Double.isFinite(altitudeMSL)) altitudeMSL = 0.0;

        final double mach = AirDensity.machFromV(speed, altitudeMSL);
        final double dep = clamp01(deploymentFrac);
        final double FN = getAirbrakeDeltaDragN(mach, dep);

        LOG.trace("calculateDragForce: M={} dep={} → DeltaDrag={} N (mode={}, speed={} m/s, altMSL={} m)",
                mach, dep, FN, lastEvaluation.mode(), speed, altitudeMSL);
        return FN;
    }

    public GenericFunction2D.Evaluation getLastEvaluation() {
        return lastEvaluation;
    }

    /** @return true if the Drag surface is loaded. */
    public boolean isReady() {
        return dragSurface != null;
    }

    // Local Helpers

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
