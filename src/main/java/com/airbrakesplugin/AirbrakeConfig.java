package com.airbrakesplugin;

import java.util.Optional;

/**
 * Configuration container for the air-brake plugin.
 * <p>
 *   Updated for the <b>bang-bang + apogee-predictor</b> control scheme.
 *   <ul>
 *     <li>PID gains kept for backward compatibility but marked {@code @Deprecated}.</li>
 *     <li>Added <i>apogeeToleranceMeters</i> so the GUI can expose the ±dead-band.</li>
 *     <li>Renamed accessors to match reflection calls in {@link AirbrakeController}
 *         (notably <code>getAlwaysOpenPercentage()</code>).</li>
 *   </ul>
 * </p>
 */
public class AirbrakeConfig {

    // ── Core aerodynamic & deployment parameters ────────────────────────────
    private String  cfdDataFilePath;
    private double  referenceArea;
    private double  referenceLength;
    private double  maxDeploymentRate;

    // Target & safety gates
    private double  targetApogee;
    private double  deployAltitudeThreshold;
    private double  maxMachForDeployment;

    // ── Bang-bang controller options ─────────────────────────────────────────
    private boolean alwaysOpenMode;
    private double  alwaysOpenPercentage;

    /** ±dead-band around set-point [m]; if {@code null} → default in controller. */
    private Double  apogeeToleranceMeters;
    
    /**
     * Constructor with default values.
     */
    public AirbrakeConfig() {
        this.referenceArea = 0.0;            // m²
        this.referenceLength = 0.0;          // m
        this.maxDeploymentRate = 5.0;        // 1/s (fraction per second)
        this.targetApogee = 0;           // m AGL
        this.deployAltitudeThreshold = 0.0;   // m AGL – prevent ground tests
        this.maxMachForDeployment = 0.0;     // cap for supersonic
        this.alwaysOpenMode = false;
        this.alwaysOpenPercentage = 1.0;       // 0–1
        this.apogeeToleranceMeters = null;
    }

    // =====================================================================
    // Getters & setters (reflection-friendly naming)
    // =====================================================================
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

    // ── Bang-bang related ──────────────────────────────────────────────────
    public boolean isAlwaysOpenMode()                { return alwaysOpenMode; }
    public void    setAlwaysOpenMode(boolean b)      { this.alwaysOpenMode = b; }

    /** Alias used by controller via reflection. */
    public double  getAlwaysOpenPercentage()         { return alwaysOpenPercentage; }
    public void    setAlwaysOpenPercentage(double pct){ this.alwaysOpenPercentage = clamp01(pct); }

    /** Optional tolerance accessor (controller calls via reflection). */
    public Optional<Double> getApogeeToleranceMeters(){ return Optional.ofNullable(apogeeToleranceMeters); }
    public void setApogeeToleranceMeters(Double tol) { this.apogeeToleranceMeters = tol; }

    // =====================================================================
    private static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }

    @Override
    public String toString() {
        return "AirbrakeConfig{" +
                "cfdDataFilePath='" + cfdDataFilePath + '\'' +
                ", referenceArea=" + referenceArea +
                ", referenceLength=" + referenceLength +
                ", targetApogee=" + targetApogee +
                ", deployAltitudeThreshold=" + deployAltitudeThreshold +
                ", maxMachForDeployment=" + maxMachForDeployment +
                ", alwaysOpenMode=" + alwaysOpenMode +
                ", alwaysOpenPercentage=" + alwaysOpenPercentage +
                ", apogeeToleranceMeters=" + apogeeToleranceMeters +
                "}";
    }
}