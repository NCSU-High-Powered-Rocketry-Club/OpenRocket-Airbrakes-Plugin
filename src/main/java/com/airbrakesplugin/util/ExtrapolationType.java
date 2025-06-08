package com.airbrakesplugin.util;

/**
 * How to handle inputs outside the original data range.
 */
public enum ExtrapolationType {
    CONSTANT,   // clamp to nearest edge
    ZERO,       // return 0.0 outside
    NATURAL     // let the interpolator extrapolate
}
