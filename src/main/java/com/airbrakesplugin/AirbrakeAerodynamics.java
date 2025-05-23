package com.airbrakesplugin; 

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolator; 
import org.apache.commons.math3.analysis.BivariateFunction; 
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
    private static final Logger log = LoggerFactory.getLogger(AirbrakeAerodynamics.class);

    private BivariateFunction cdFunction;
    private BivariateFunction cmFunction;
    private double[] machPoints; 
    private double[] deploymentPoints; 

    public AirbrakeAerodynamics(String csvFilePath) throws IOException {
        if (csvFilePath == null || csvFilePath.trim().isEmpty()) {
            throw new IOException("CSV file path cannot be null or empty.");
        }
        List<AirbrakeDataPoint> dataPoints = loadCFDDataInternal(csvFilePath);
        prepareInterpolators(dataPoints);
    }

    private List<AirbrakeDataPoint> loadCFDDataInternal(String csvFilePath) throws IOException {
        log.info("Loading CFD data from: {}", csvFilePath);
        List<AirbrakeDataPoint> dataPoints = new ArrayList<>();
        Reader reader = null;
        try {
            reader = new FileReader(csvFilePath);
            log.debug("Successfully opened {} from filesystem.", csvFilePath);
        } catch (IOException e_fs) {
            log.warn("Could not open {} from filesystem (Error: {}), trying as resource.", csvFilePath, e_fs.getMessage());
            InputStream is = getClass().getClassLoader().getResourceAsStream(csvFilePath);
            if (is == null) { 
                is = Thread.currentThread().getContextClassLoader().getResourceAsStream(csvFilePath);
            }
            if (is != null) {
                reader = new InputStreamReader(is);
                log.debug("Successfully opened {} as resource.", csvFilePath);
            } else {
                log.error("Failed to find CFD data file: {} either in filesystem or as resource.", csvFilePath);
                throw new IOException("CFD data file not found: " + csvFilePath, e_fs); 
            }
        }
        
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
                    double deployment = Double.parseDouble(record.get("DeploymentPercentage")) / 100.0;
                    double cdIncrement = Double.parseDouble(record.get("Cd_increment"));
                    double cmIncrement = Double.parseDouble(record.get("Cm_increment"));
                    dataPoints.add(new AirbrakeDataPoint(mach, deployment, cdIncrement, cmIncrement));
                } catch (NumberFormatException nfe) {
                    log.warn("Skipping malformed CSV record (number format): {} - Error: {}", record.toMap(), nfe.getMessage());
                } catch (IllegalArgumentException iae) { 
                    log.warn("Issue with CSV record (e.g. missing header for column): {} - Error: {}. Ensure CSV has headers: Mach,DeploymentPercentage,Cd_increment,Cm_increment", record.toMap(), iae.getMessage());
                }
            }
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException e_close) {
                    log.warn("Error closing CSV reader: {}", e_close.getMessage());
                }
            }
        }
        log.info("Loaded {} data points from CSV.", dataPoints.size());
        if (dataPoints.isEmpty()) {
            throw new IOException("No valid data points loaded from CFD file: " + csvFilePath + ". Ensure file is not empty and data is correctly formatted.");
        }
        return dataPoints;
    }

    private void prepareInterpolators(List<AirbrakeDataPoint> dataPoints) throws IOException {
        if (dataPoints == null || dataPoints.isEmpty()) {
             throw new IOException("Cannot prepare interpolators with no data points.");
        }
        this.machPoints = dataPoints.stream().mapToDouble(AirbrakeDataPoint::getMach).distinct().sorted().toArray();
        this.deploymentPoints = dataPoints.stream().mapToDouble(AirbrakeDataPoint::getDeploymentPercentage).distinct().sorted().toArray();

        if (machPoints.length < 2 || deploymentPoints.length < 2) {
            log.error("Insufficient distinct Mach ({}) or Deployment ({}) points for 2D interpolation. Need at least 2 of each.",
                      machPoints.length, deploymentPoints.length);
            throw new IOException("Insufficient distinct data points for interpolation." +
                                  " Mach points: " + machPoints.length + ", Deployment points: " + deploymentPoints.length +
                                  ". Bicubic interpolation requires at least 2x2 grid.");
        }

        double[][] cdGridValues = new double[machPoints.length][deploymentPoints.length];
        double[][] cmGridValues = new double[machPoints.length][deploymentPoints.length];

        for (int i = 0; i < machPoints.length; i++) {
            for (int j = 0; j < deploymentPoints.length; j++) {
                double currentMach = machPoints[i];
                double currentDeployment = deploymentPoints[j];
                AirbrakeDataPoint dp = dataPoints.stream()
                    .filter(p -> Math.abs(p.getMach() - currentMach) < 1e-9 && Math.abs(p.getDeploymentPercentage() - currentDeployment) < 1e-9)
                    .findFirst()
                    .orElse(null);
                if (dp != null) {
                    cdGridValues[i][j] = dp.getCdIncrement();
                    cmGridValues[i][j] = dp.getCmIncrement();
                } else {
                    log.warn("Missing data point for Mach={}, Deployment={}. This might cause issues with interpolation. Setting to NaN.", currentMach, currentDeployment);
                    cdGridValues[i][j] = Double.NaN; 
                    cmGridValues[i][j] = Double.NaN;
                }
            }
        }
        
        try {
            BicubicInterpolator interpolator = new BicubicInterpolator();
            cdFunction = interpolator.interpolate(machPoints, deploymentPoints, cdGridValues);
            cmFunction = interpolator.interpolate(machPoints, deploymentPoints, cmGridValues);
            log.info("BicubicInterpolatingFunctions prepared for Cd and Cm.");
        } catch (Exception e) { 
            log.error("Failed to create BicubicInterpolatingFunction: {}", e.getMessage(), e);
            cdFunction = null;
            cmFunction = null;
            throw new IOException("Failed to prepare interpolators due to: " + e.getMessage(), e);
        }
    }

    public double getIncrementalCd(double mach, double deploymentPercentage) {
        if (cdFunction == null || machPoints == null || deploymentPoints == null) {
            log.warn("CdFunction not available or data axes not loaded. Returning 0.0 for Cd. Mach: {}, Depl: {}", mach, deploymentPercentage);
            return 0.0; 
        }
        mach = clamp(mach, machPoints[0], machPoints[machPoints.length - 1]);
        deploymentPercentage = clamp(deploymentPercentage, deploymentPoints[0], deploymentPoints[deploymentPoints.length - 1]);
        
        try {
            return cdFunction.value(mach, deploymentPercentage);
        } catch (Exception e) { 
            log.error("Error during Cd (bicubic) interpolation for Mach={}, Deployment={}: {}", mach, deploymentPercentage, e.getMessage());
            return Double.NaN; 
        }
    }

    public double getIncrementalCm(double mach, double deploymentPercentage) {
         if (cmFunction == null || machPoints == null || deploymentPoints == null) {
            log.warn("CmFunction not available or data axes not loaded. Returning 0.0 for Cm. Mach: {}, Depl: {}", mach, deploymentPercentage);
            return 0.0; 
        }
        mach = clamp(mach, machPoints[0], machPoints[machPoints.length - 1]);
        deploymentPercentage = clamp(deploymentPercentage, deploymentPoints[0], deploymentPoints[deploymentPoints.length - 1]);

        try {
            return cmFunction.value(mach, deploymentPercentage);
        } catch (Exception e) {
            log.error("Error during Cm (bicubic) interpolation for Mach={}, Deployment={}: {}", mach, deploymentPercentage, e.getMessage());
            return Double.NaN;
        }
    }

    private double clamp(double value, double min, double max) {
        if (min > max) { 
            log.warn("Clamp min ({}) is greater than max ({}). This indicates an issue with data points. Returning value un-clamped.", min, max);
            return value;
        }
        return Math.max(min, Math.min(max, value));
    }

    private static class AirbrakeDataPoint {
        private final double mach;
        private final double deploymentPercentage; 
        private final double cdIncrement;
        private final double cmIncrement;
        public AirbrakeDataPoint(double mach, double deploymentPercentage, double cdIncrement, double cmIncrement) {
            this.mach = mach;
            this.deploymentPercentage = deploymentPercentage;
            this.cdIncrement = cdIncrement;
            this.cmIncrement = cmIncrement;
        }
        public double getMach() { return mach; }
        public double getDeploymentPercentage() { return deploymentPercentage; }
        public double getCdIncrement() { return cdIncrement; }
        public double getCmIncrement() { return cmIncrement; }
    }
}