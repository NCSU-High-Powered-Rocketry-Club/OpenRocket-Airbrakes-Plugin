package com.airbrakesplugin.util;

/**
 * Runtime policy for Mach values outside the loaded CFD force table.
 */
public enum HighMachExtrapolationMode {
    /** Use a bounded nonlinear projection from the nearest two Mach stations. */
    BOUNDED_NONLINEAR,
    /** Return zero aerodynamic delta outside the validated table bounds. */
    FAIL_CLOSED,
    /** Clamp to the nearest table edge, matching the legacy behavior. */
    CONSTANT;

    public static HighMachExtrapolationMode fromString(String raw) {
        if (raw == null || raw.isBlank()) {
            return BOUNDED_NONLINEAR;
        }
        try {
            return HighMachExtrapolationMode.valueOf(raw.trim().toUpperCase());
        } catch (IllegalArgumentException ignored) {
            return BOUNDED_NONLINEAR;
        }
    }
}
