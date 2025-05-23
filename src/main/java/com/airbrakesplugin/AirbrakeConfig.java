package com.airbrakesplugin; 

public class AirbrakeConfig {
    private String cfdDataFilePath = "airbrakes_cfd_data.csv";
    private double referenceArea = 0.01266; 
    private double referenceLength = 0.1016; 
    private double maxDeploymentRate = 0.5; 
    private double targetApogee = 3000; 
    private double deployAltitudeThreshold = 500; 
    private double maxMachForDeployment = 0.9; 

    public String getCfdDataFilePath() { return cfdDataFilePath; }
    public void setCfdDataFilePath(String cfdDataFilePath) { this.cfdDataFilePath = cfdDataFilePath; }
    public double getReferenceArea() { return referenceArea; }
    public void setReferenceArea(double referenceArea) { this.referenceArea = referenceArea; }
    public double getReferenceLength() { return referenceLength; }
    public void setReferenceLength(double referenceLength) { this.referenceLength = referenceLength; }
    public double getMaxDeploymentRate() { return maxDeploymentRate; }
    public void setMaxDeploymentRate(double maxDeploymentRate) { this.maxDeploymentRate = maxDeploymentRate; }
    public double getTargetApogee() { return targetApogee; }
    public void setTargetApogee(double targetApogee) { this.targetApogee = targetApogee; }
    public double getDeployAltitudeThreshold() { return deployAltitudeThreshold; }
    public void setDeployAltitudeThreshold(double deployAltitudeThreshold) { this.deployAltitudeThreshold = deployAltitudeThreshold; }
    public double getMaxMachForDeployment() { return maxMachForDeployment; }
    public void setMaxMachForDeployment(double maxMachForDeployment) { this.maxMachForDeployment = maxMachForDeployment; }

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