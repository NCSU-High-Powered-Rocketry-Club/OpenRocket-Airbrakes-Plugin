package com.airbrakesplugin;

/**
 * Configuration container for the air-brake plugin.
 * All values are set via the configurator UI (or defaulted here).
 */
public class AirbrakeConfig {

    // --- Existing parameters --------------------------------------------------
    private String  cfdDataFilePath;
    private double  referenceArea;
    private double  referenceLength;
    private double  maxDeploymentRate;
    private double  targetApogee;
    private double  deployAltitudeThreshold;
    private double  maxMachForDeployment;

    // --- NEW Waterloo-style controller parameters -----------------------------
    /** If true, the controller holds the brakes at {@link #alwaysOpenPercent}. */
    private boolean alwaysOpenMode   = false;
    /** Fixed deployment level for “always-open” mode (0–1). */
    private double  alwaysOpenPercent = 0.40;

    /** PID gains – tuned for typical HPR rockets. */
    private double kp = 0.010;     // proportional [-]
    private double ki = 0.0004;    // integral     [1/s]
    private double kd = 0.040;     // derivative   [s]

    // -------------------------------------------------------------------------
    // Getters & setters (existing + new)
    // -------------------------------------------------------------------------
    public String getCfdDataFilePath()               { return cfdDataFilePath; }
    public void   setCfdDataFilePath(String path)    { this.cfdDataFilePath = path; }

    public double getReferenceArea()                 { return referenceArea; }
    public void   setReferenceArea(double area)      { this.referenceArea = area; }

    public double getReferenceLength()               { return referenceLength; }
    public void   setReferenceLength(double len)     { this.referenceLength = len; }

    public double getMaxDeploymentRate()             { return maxDeploymentRate; }
    public void   setMaxDeploymentRate(double rate)  { this.maxDeploymentRate = rate; }

    public double getTargetApogee()                  { return targetApogee; }
    public void   setTargetApogee(double apogee)     { this.targetApogee = apogee; }

    public double getDeployAltitudeThreshold()       { return deployAltitudeThreshold; }
    public void   setDeployAltitudeThreshold(double h){ this.deployAltitudeThreshold = h; }

    public double getMaxMachForDeployment()          { return maxMachForDeployment; }
    public void   setMaxMachForDeployment(double m)  { this.maxMachForDeployment = m; }

    // --- NEW fields -----------------------------------------------------------
    public boolean isAlwaysOpenMode()                { return alwaysOpenMode; }
    public void    setAlwaysOpenMode(boolean b)      { this.alwaysOpenMode = b; }

    public double  getAlwaysOpenPercent()            { return alwaysOpenPercent; }
    public void    setAlwaysOpenPercent(double pct)  { this.alwaysOpenPercent = clamp01(pct); }

    public double  getKp()                           { return kp; }
    public void    setKp(double kp)                  { this.kp = kp; }

    public double  getKi()                           { return ki; }
    public void    setKi(double ki)                  { this.ki = ki; }

    public double  getKd()                           { return kd; }
    public void    setKd(double kd)                  { this.kd = kd; }

    // -------------------------------------------------------------------------
    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }

    @Override
    public String toString() {
        return "AirbrakeConfig{" +
               "cfdDataFilePath='" + cfdDataFilePath + '\'' +
               ", referenceArea="          + referenceArea +
               ", referenceLength="        + referenceLength +
               ", maxDeploymentRate="      + maxDeploymentRate +
               ", targetApogee="           + targetApogee +
               ", deployAltitudeThreshold="+ deployAltitudeThreshold +
               ", maxMachForDeployment="   + maxMachForDeployment +
               ", alwaysOpenMode="         + alwaysOpenMode +
               ", alwaysOpenPercent="      + alwaysOpenPercent +
               ", kp=" + kp + ", ki=" + ki + ", kd=" + kd +
               '}';
    }
}
