package com.airbrakesplugin;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.FlightDataType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * PID-style air-brake controller (Waterloo Rocketry inspired).
 * Coast-phase detection is done via THRUST=0 instead of a nonexistent
 * SimulationStatus.isCoastPhase() helper.
 */
public class AirbrakeController {

    private static final Logger log = LoggerFactory.getLogger(AirbrakeController.class);

    // ── Configuration ─────────────────────────────────────────────────────────
    private final AirbrakeConfig config;
    private final boolean alwaysOpenMode;
    private final double  alwaysOpenPct;
    private final double  kp, ki, kd;

    // Integrator state
    private double iErr   = 0.0;
    private double prevErr = 0.0;
    private double prevT   = Double.NaN;

    private static final double I_MIN = -0.5, I_MAX = 0.5;

    public AirbrakeController(AirbrakeConfig cfg) {
        if (cfg == null) {
            log.error("AirbrakeController initialised with null config – using defaults.");
            cfg = new AirbrakeConfig();
        }
        this.config         = cfg;
        this.alwaysOpenMode = cfg.isAlwaysOpenMode();
        this.alwaysOpenPct  = clamp01(cfg.getAlwaysOpenPercent());
        this.kp = cfg.getKp() != 0 ? cfg.getKp() : 0.010;
        this.ki = cfg.getKi() != 0 ? cfg.getKi() : 0.0004;
        this.kd = cfg.getKd() != 0 ? cfg.getKd() : 0.040;
    }

    /**
     * Compute the commanded deployment (0–1).
     */
    public double getCommandedDeployment(double alt, double vZ, double mach,
                                         double currentDeploy, SimulationStatus status) {

        // ── 1  Fixed “always-open” mode ──────────────────────────────────────
        if (alwaysOpenMode) return alwaysOpenPct;

        // ── 2  Determine whether the motor is still burning ─────────────────
        boolean powered = false;
        if (status != null) {
            try {
                double thrust = status.getFlightData().getLast(FlightDataType.TYPE_THRUST_FORCE);
                powered = thrust > 1.0;               // >1 N ⇒ powered phase
            } catch (Exception e) {
                log.debug("TYPE_THRUST unavailable – assuming unpowered.");
            }
        }

        // ── 3  Guard-clauses (disable brakes) ───────────────────────────────
        if (status == null || powered ||
            vZ <= 0 ||
            alt < config.getDeployAltitudeThreshold() ||
            (config.getMaxMachForDeployment() > 0 &&
             mach > config.getMaxMachForDeployment())) {
            resetIntegrator(status);
            return 0.0;
        }

        // ── 4  PID on ballistic apogee prediction ĥ = h + v²/(2g) ──────────
        double t  = status.getSimulationTime();
        if (Double.isNaN(prevT)) prevT = t;
        double dt = Math.max(t - prevT, 1e-3);

        final double g = 9.80665;
        double hPred   = alt + vZ * vZ / (2.0 * g);
        double err     = hPred - config.getTargetApogee();

        double p = kp * err;

        iErr += err * dt;
        iErr  = clamp(iErr, I_MIN / ki, I_MAX / ki);
        double i = ki * iErr;

        double d = kd * (err - prevErr) / dt;

        double raw = currentDeploy + (p + i + d);
        double cmd = clamp01(raw);

        // ── 5  Actuator rate-limit ──────────────────────────────────────────
        double maxΔ = config.getMaxDeploymentRate() * dt;
        cmd = clamp(cmd, currentDeploy - maxΔ, currentDeploy + maxΔ);

        prevErr = err;
        prevT   = t;
        return cmd;
    }

    // ── Utility helpers ─────────────────────────────────────────────────────
    private static double clamp01(double v)            { return clamp(v, 0, 1); }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private void resetIntegrator(SimulationStatus s) {
        iErr = 0.0;
        if (s != null) prevT = s.getSimulationTime();
    }
}
