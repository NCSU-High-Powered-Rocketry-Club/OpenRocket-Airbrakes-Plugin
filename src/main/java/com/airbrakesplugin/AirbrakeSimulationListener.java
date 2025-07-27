package com.airbrakesplugin;

import com.airbrakesplugin.util.AirDensity;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Air‑brake 6‑DoF simulation listener – OpenRocket 23.09
 *
 * <p>Only *incremental coefficients* (ΔCD, ΔCm) are injected; all other
 * dimensional work is left to OpenRocket.  We therefore mutate the incoming
 * {@link AerodynamicForces} instance in‑place instead of creating a new one –
 * this guarantees every other field (CN, CY, damping derivatives, etc.) stays
 * fully initialised and finite.</p>
 */
public final class AirbrakeSimulationListener extends AbstractSimulationListener {

    // ─────────────────────────────────────────────────────────────────────
    private static final Logger LOG = LoggerFactory.getLogger(AirbrakeSimulationListener.class);

    private final AirbrakeConfig config;
    private final Rocket         rocket;

    private AirbrakeAerodynamics aerodynamics;
    private AirbrakeController   controller;

    private final double referenceArea;      // m²
    private final double referenceLength;    // m
    private final double maxRate;            // %·s⁻¹

    private double currentDeployment = 0.0;  // 0‑1
    private double lastTime          = 0.0;  // s

    // ─────────────────────────────────────────────────────────────────────
    public AirbrakeSimulationListener(final AirbrakeConfig cfg, final Rocket rocket) {
        if (cfg == null) throw new IllegalArgumentException("AirbrakeConfig null");
        this.config  = cfg;
        this.rocket  = rocket;

        /* Reference geometry fallback chain */
        final double rocketSref = rocket != null ? Math.PI * Math.pow(rocket.getBoundingRadius(), 2) : 0.0;
        referenceArea   = cfg.getReferenceArea()   > 0 ? cfg.getReferenceArea()   : (rocketSref > 0 ? rocketSref : Math.PI * 0.08 * 0.08);
        referenceLength = cfg.getReferenceLength() > 0 ? cfg.getReferenceLength() : (rocket != null ? rocket.getLength() : 0.16);
        maxRate         = cfg.getMaxDeploymentRate();

        LOG.info("AirbrakeListener: Sref={} m²  Lref={} m  rate={} %/s", referenceArea, referenceLength, maxRate);
    }

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public void startSimulation(final SimulationStatus st) throws SimulationException {
        aerodynamics      = new AirbrakeAerodynamics(config.getCfdDataFilePath());
        controller        = new AirbrakeController(config);
        currentDeployment = 0.0;
        lastTime          = st.getSimulationTime();
        LOG.info("Airbrake listener initialised");
        LOG.debug("Initial state: deployment={}, time={}", currentDeployment, lastTime);
    }

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public boolean preStep(final SimulationStatus st) {
        if (aerodynamics == null || controller == null) return true;
        final double t  = st.getSimulationTime();
        final double dt = t - lastTime;
        lastTime        = t;
        if (dt <= 0) return true;

        LOG.debug("preStep at t={}: dt={}", t, dt);
        
        final double alt  = st.getRocketPosition().z;
        final double vz   = st.getRocketVelocity().z;
        final double mach = st.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        LOG.debug("Flight state: altitude={} m, velocity_z={} m/s, mach={}, currentDeployment={}", 
                 alt, vz, mach, currentDeployment);
        
        final double cmd  = controller.getCommandedDeployment(alt, vz, mach, currentDeployment, st);
        LOG.debug("Controller commanded deployment: {}", cmd);

        final double dMax = maxRate * dt;
        double newDeployment = clamp(cmd, currentDeployment - dMax, currentDeployment + dMax);
        LOG.debug("Deployment update: max_rate_change={}, new_deployment={}", dMax, newDeployment);
        
        currentDeployment = newDeployment;
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public AerodynamicForces postAerodynamicCalculation(final SimulationStatus st,
                                                        final AerodynamicForces base) {
        if (aerodynamics == null || currentDeployment < 1e-6) {
            LOG.debug("Skipping aero calculation: aerodynamics={}, deployment={}", 
                     aerodynamics != null ? "initialized" : "null", currentDeployment);
            return base;
        }

        final double mach = st.getFlightData().getLast(FlightDataType.TYPE_MACH_NUMBER);
        LOG.debug("postAero calculation: mach={}, deployment={}", mach, currentDeployment);
        
        final double dCd  = aerodynamics.getIncrementalCd(mach, currentDeployment);
        final double dCm  = aerodynamics.getIncrementalCm(mach, currentDeployment);
        LOG.debug("Calculated incremental coefficients: dCd={}, dCm={}", dCd, dCm);
        
        if (!Double.isFinite(dCd) || !Double.isFinite(dCm)) {
            LOG.warn("ΔC non‑finite – M={}  dCd={}  dCm={} (skipped)", mach, dCd, dCm);
            return base;
        }

        /* mutate in‑place → preserves every other coefficient that OR expects */
        double originalCD = base.getCD();
        double originalCm = base.getCm();
        base.setCD(originalCD + dCd);
        base.setCm(originalCm + dCm);
        LOG.debug("Updated coefficients: CD: {} → {}, Cm: {} → {}", 
                 originalCD, base.getCD(), originalCm, base.getCm());
        
        return base;
    }

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public void endSimulation(final SimulationStatus st, final SimulationException ex) {
        if (ex != null) LOG.error("Simulation ended with error", ex);
        else            LOG.info("Simulation complete – final deployment {}", currentDeployment);
        
        LOG.debug("Final simulation state - time: {}, deployment: {}", 
                 st != null ? st.getSimulationTime() : "unknown", currentDeployment);
    }

    // ─────────────────────────────────────────────────────────────────────
    private static double clamp(double val, double lo, double hi) {
        return Math.min(Math.max(val, lo), hi);
    }
}
