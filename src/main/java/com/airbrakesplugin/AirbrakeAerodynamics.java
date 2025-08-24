package com.airbrakesplugin;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.math3.analysis.BivariateFunction;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.List;

public class AirbrakeAerodynamics {

    private static final Logger LOG = LoggerFactory.getLogger(AirbrakeAerodynamics.class);

    /**
     * Anything below this threshold is treated as CLOSED (0),
     * anything at/above is treated as FULLY OPEN (1).
     * Keep near 1.0 to only grant "open" when actuator is essentially at its limit.
     */
    private static final double OPEN_THRESHOLD = 0.999;

    /** Tiny offset used to nudge values inside the interpolation domain. */
    private static final double EPS_FRACTION = 1e-9;

    private final BivariateFunction cdFunction;
    private final BivariateFunction cmFunction;
    private final double[] machPoints;
    private final double[] deploymentPoints;

    public AirbrakeAerodynamics(String csvFilePath) {
        if (csvFilePath == null || csvFilePath.isBlank())
            throw new IllegalArgumentException("CSV file path is null/empty");

        try {
            List<AirbrakeDataPoint> dataPoints = loadCFDData(csvFilePath);
            machPoints = extractDistinctSortedValues(dataPoints, AirbrakeDataPoint::getMach);
            deploymentPoints = extractDistinctSortedValues(dataPoints, AirbrakeDataPoint::getDeploymentPercentage);

            validateDataPoints();

            double[][] cdGridValues = buildValueGrid(dataPoints, AirbrakeDataPoint::getCdIncrement);
            double[][] cmGridValues = buildValueGrid(dataPoints, AirbrakeDataPoint::getCmIncrement);

            BicubicInterpolator interpolator = new BicubicInterpolator();
            cdFunction = interpolator.interpolate(machPoints, deploymentPoints, cdGridValues);
            cmFunction = interpolator.interpolate(machPoints, deploymentPoints, cmGridValues);

            LOG.info("Airbrake CFD data loaded with {} Mach points, {} deployment points",
                    machPoints.length, deploymentPoints.length);
        } catch (IOException e) {
            LOG.error("Failed to load airbrake CFD data: {}", e.getMessage());
            throw new IllegalArgumentException("Failed to load airbrake CFD data", e);
        }
    }

    // -------------------------------------------------------------------------
    // Public API (binary endpoints, robust to NaN/out-of-range inputs)
    // -------------------------------------------------------------------------

    /** Returns ΔCd at the current Mach; deployment coerced to CLOSED or OPEN grid endpoints. */
    public double getIncrementalCd(double mach, double deployFrac) {
        try {
            final double mEval = sanitizeMach(mach);
            final double dEval = pickBinaryDeploymentValue(deployFrac);
            final double val   = safeEvaluate(cdFunction, mEval, dEval);
            return Double.isFinite(val) ? val : 0.0;
        } catch (Exception e) {
            LOG.warn("Cd interpolation (binary deploy) failed: mach={}, deployIn={}, error={}",
                    mach, deployFrac, e.getMessage());
            return 0.0;
        }
    }

    /** Returns ΔCm at the current Mach; deployment coerced to CLOSED or OPEN grid endpoints. */
    public double getIncrementalCm(double mach, double deployFrac) {
        try {
            final double mEval = sanitizeMach(mach);
            final double dEval = pickBinaryDeploymentValue(deployFrac);
            final double val   = safeEvaluate(cmFunction, mEval, dEval);
            return Double.isFinite(val) ? val : 0.0;
        } catch (Exception e) {
            LOG.warn("Cm interpolation (binary deploy) failed: mach={}, deployIn={}, error={}",
                    mach, deployFrac, e.getMessage());
            return 0.0;
        }
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * Map any deployment fraction to one of the two endpoints in the CFD table:
     * CLOSED -> deploymentPoints[0], OPEN -> deploymentPoints[last].
     * NaN → CLOSED endpoint.
     */
    private double pickBinaryDeploymentValue(double deployFrac) {
        if (!Double.isFinite(deployFrac)) {
            LOG.trace("Deployment {} treated as CLOSED (NaN/non-finite).", deployFrac);
            return deploymentPoints[0];
        }
        final boolean open = (deployFrac >= OPEN_THRESHOLD);
        final double chosen = open ? deploymentPoints[deploymentPoints.length - 1] : deploymentPoints[0];

        // Informative (non-spammy) traces when mid-values are coerced.
        if (!open && deployFrac > 0.0 && deployFrac < OPEN_THRESHOLD) {
            LOG.trace("Deployment {} coerced to CLOSED endpoint {}", String.format("%.3f", deployFrac), String.format("%.3f", chosen));
        } else if (open && deployFrac < 1.0) {
            LOG.trace("Deployment {} coerced to OPEN endpoint {}", String.format("%.3f", deployFrac), String.format("%.3f", chosen));
        }
        return chosen;
    }

    /**
     * Ensure Mach is finite and strictly within the interpolation domain.
     * NaN/out-of-range → nearest bound, then nudged slightly inside.
     */
    private double sanitizeMach(double mach) {
        final double min = machPoints[0];
        final double max = machPoints[machPoints.length - 1];

        if (!Double.isFinite(mach)) {
            // Default to lower bound when NaN; matches your warning line case.
            final double nudged = nudgeInside(min, min, max);
            LOG.trace("Mach {} sanitized to {}", mach, nudged);
            return nudged;
        }
        if (mach <= min) return nudgeInside(min, min, max);
        if (mach >= max) return nudgeInside(max, min, max);
        return mach;
    }

    /** Evaluate f(x,y) with a small nudge inside the domain to avoid edge pathologies. */
    private double safeEvaluate(BivariateFunction f, double mach, double deployValue) {
        // Deployment value is guaranteed to be one of the exact grid endpoints.
        // We only ever nudge Mach (x), not deployment (y), to preserve binary behavior.
        final double min = machPoints[0];
        final double max = machPoints[machPoints.length - 1];

        // Try at the given mach; if it fails or returns NaN, try tiny nudges inward.
        double x = mach;
        for (int i = 0; i < 3; i++) {
            try {
                double val = f.value(x, deployValue);
                if (Double.isFinite(val)) return val;
            } catch (Exception ignore) {
                // fall through to nudge
            }
            // Nudge inward proportionally to domain width
            x = nudgeInside(x, min, max);
        }
        // Give up gracefully
        LOG.debug("Interpolation failed at mach={} (sanitized from {}), deploy={}; returning 0",
                x, mach, deployValue);
        return 0.0;
    }

    /** Returns a value strictly inside (min,max), or at boundaries slightly offset inward. */
    private double nudgeInside(double x, double min, double max) {
        final double w = Math.max(1.0, Math.abs(max - min));
        final double eps = EPS_FRACTION * w;

        if (x <= min) return Math.min(min + eps, max - eps);
        if (x >= max) return Math.max(max - eps, min + eps);
        // already inside: pull slightly away from the closest bound
        final double dMin = Math.abs(x - min);
        final double dMax = Math.abs(max - x);
        if (dMin < dMax) return Math.min(x + eps, max - eps);
        return Math.max(x - eps, min + eps);
    }

    private List<AirbrakeDataPoint> loadCFDData(String csvFilePath) throws IOException {
        LOG.info("Loading CFD data from: {}", csvFilePath);
        List<AirbrakeDataPoint> dataPoints = new ArrayList<>();

        Reader reader = openReader(csvFilePath);
        CSVFormat csvFormat = CSVFormat.DEFAULT.builder()
                .setHeader("Mach", "DeploymentPercentage", "Cd_increment", "Cm_increment")
                .setSkipHeaderRecord(true)
                .setTrim(true)
                .setCommentMarker('#')
                .setIgnoreEmptyLines(true)
                .build();

        try (CSVParser parser = new CSVParser(reader, csvFormat)) {
            for (CSVRecord record : parser) {
                try {
                    double mach = Double.parseDouble(record.get("Mach"));
                    double deploy = Double.parseDouble(record.get("DeploymentPercentage")) / 100.0;
                    double cdIncr = Double.parseDouble(record.get("Cd_increment"));
                    double cmIncr = Double.parseDouble(record.get("Cm_increment"));
                    dataPoints.add(new AirbrakeDataPoint(mach, deploy, cdIncr, cmIncr));
                } catch (Exception e) {
                    LOG.warn("Skipping malformed CSV record {}: {}",
                            record.getRecordNumber(), e.getMessage());
                }
            }
        }

        LOG.info("Loaded {} data points from {}", dataPoints.size(), csvFilePath);
        if (dataPoints.isEmpty()) {
            throw new IOException("No valid CFD data points loaded from: " + csvFilePath);
        }

        return dataPoints;
    }

    private Reader openReader(String csvFilePath) throws IOException {
        try {
            Reader reader = new FileReader(csvFilePath);
            LOG.debug("Opened {} from filesystem", csvFilePath);
            return reader;
        } catch (IOException e) {
            LOG.debug("Trying to load {} as resource", csvFilePath);

            InputStream is = getClass().getClassLoader().getResourceAsStream(csvFilePath);
            if (is == null) {
                is = Thread.currentThread().getContextClassLoader().getResourceAsStream(csvFilePath);
            }

            if (is != null) {
                LOG.debug("Opened {} as classpath resource", csvFilePath);
                return new InputStreamReader(is);
            } else {
                throw new IOException("CFD data file not found: " + csvFilePath);
            }
        }
    }

    private double[] extractDistinctSortedValues(List<AirbrakeDataPoint> dataPoints,
                                                 java.util.function.Function<AirbrakeDataPoint, Double> valueExtractor) {
        return dataPoints.stream()
                .map(valueExtractor)
                .distinct()
                .sorted()
                .mapToDouble(Double::doubleValue)
                .toArray();
    }

    private void validateDataPoints() throws IOException {
        if (machPoints.length < 2 || deploymentPoints.length < 2) {
            throw new IOException(String.format(
                    "Insufficient distinct data points for interpolation. Found %d Mach points and %d deployment points.",
                    machPoints.length, deploymentPoints.length));
        }
    }

    private double[][] buildValueGrid(List<AirbrakeDataPoint> dataPoints,
                                      java.util.function.Function<AirbrakeDataPoint, Double> valueExtractor) throws IOException {
        double[][] grid = new double[machPoints.length][deploymentPoints.length];

        for (int i = 0; i < machPoints.length; i++) {
            for (int j = 0; j < deploymentPoints.length; j++) {
                double m = machPoints[i];
                double d = deploymentPoints[j];

                AirbrakeDataPoint point = findDataPoint(dataPoints, m, d);
                if (point == null) {
                    throw new IOException(String.format(
                            "Missing data point for Mach=%.3f, Deploy=%.3f", m, d));
                }

                grid[i][j] = valueExtractor.apply(point);
            }
        }

        return grid;
    }

    private AirbrakeDataPoint findDataPoint(List<AirbrakeDataPoint> dataPoints, double mach, double deploy) {
        final double EPSILON = 1e-9;
        return dataPoints.stream()
                .filter(p -> Math.abs(p.getMach() - mach) < EPSILON
                        && Math.abs(p.getDeploymentPercentage() - deploy) < EPSILON)
                .findFirst()
                .orElse(null);
    }

    // --- unused at the moment, but handy ------------------------------------
    @SuppressWarnings("unused")
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static final class AirbrakeDataPoint {
        private final double mach;
        private final double deploymentPercentage;
        private final double cdIncrement;
        private final double cmIncrement;

        AirbrakeDataPoint(double m, double d, double cd, double cm) {
            this.mach = m;
            this.deploymentPercentage = d;
            this.cdIncrement = cd;
            this.cmIncrement = cm;
        }

        double getMach() { return mach; }
        double getDeploymentPercentage() { return deploymentPercentage; }
        double getCdIncrement() { return cdIncrement; }
        double getCmIncrement() { return cmIncrement; }
    }
}
