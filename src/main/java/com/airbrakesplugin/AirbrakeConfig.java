package com.airbrakesplugin;

/**
 * Configuration container for the airbrake plugin.
 * All values are set via the configurator UI.
 */
public class AirbrakeConfig {
    private String cfdDataFilePath;
    private double referenceArea;
    private double referenceLength;
    private double maxDeploymentRate;
    private double targetApogee;
    private double deployAltitudeThreshold;
    private double maxMachForDeployment;

    public String getCfdDataFilePath() {
        return cfdDataFilePath;
    }
    public void setCfdDataFilePath(String cfdDataFilePath) {
        this.cfdDataFilePath = cfdDataFilePath;
    }

    public double getReferenceArea() {
        return referenceArea;
    }
    public void setReferenceArea(double referenceArea) {
        this.referenceArea = referenceArea;
    }

    public double getReferenceLength() {
        return referenceLength;
    }
    public void setReferenceLength(double referenceLength) {
        this.referenceLength = referenceLength;
    }

    public double getMaxDeploymentRate() {
        return maxDeploymentRate;
    }
    public void setMaxDeploymentRate(double maxDeploymentRate) {
        this.maxDeploymentRate = maxDeploymentRate;
    }

    public double getTargetApogee() {
        return targetApogee;
    }
    public void setTargetApogee(double targetApogee) {
        this.targetApogee = targetApogee;
    }

    public double getDeployAltitudeThreshold() {
        return deployAltitudeThreshold;
    }
    public void setDeployAltitudeThreshold(double deployAltitudeThreshold) {
        this.deployAltitudeThreshold = deployAltitudeThreshold;
    }

    public double getMaxMachForDeployment() {
        return maxMachForDeployment;
    }
    public void setMaxMachForDeployment(double maxMachForDeployment) {
        this.maxMachForDeployment = maxMachForDeployment;
    }

    @Override
    public String toString() {
        return "AirbrakeConfig{" +
               "cfdDataFilePath='" + cfdDataFilePath + '\'' +
               ", referenceArea=" + referenceArea +
               ", referenceLength=" + referenceLength +
               ", maxDeploymentRate=" + maxDeploymentRate +
               ", targetApogee=" + targetApogee +
               ", deployAltitudeThreshold=" + deployAltitudeThreshold +
               ", maxMachForDeployment=" + maxMachForDeployment +
               '}';
    }
}
