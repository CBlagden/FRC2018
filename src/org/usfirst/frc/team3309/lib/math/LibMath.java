package org.usfirst.frc.team3309.lib.math;

public class LibMath {

    public static double handleDeadband(double val, double lim) {
        return Math.abs(val) < Math.abs(lim) ? 0.0 : val;
    }

    public static boolean isInThreshold(double val, double threshold) {
        return Math.abs(val) < Math.abs(threshold);
    }

    public static boolean isWithin(double val, double a, double b) {
        if (a < b) {
            return val > a && val < b;
        }
        return val < a && val > b;
    }

    /**
     * Keeps a value in a range by truncating it.
     *
     * @param toCoerce
     *            the value to coerce
     * @param high
     *            the high value of the range
     * @param low
     *            the low value of the range
     * @return the coerced value
     */
    public static double coerce(double toCoerce, double high, double low) {
        if (toCoerce > high) {
            return high;
        } else if (toCoerce < low) {
            return low;
        }
        return toCoerce;
    }

    public static double coercedNormalize(double rawValue, double minInput, double maxInput, double minOutput,
                                          double maxOutput) {
        if (rawValue < minInput) {
            return minOutput;
        } else if (rawValue > maxInput) {
            return maxOutput;
        }
        double norm = (Math.abs(rawValue) - minInput) / (maxInput - minInput);
        norm = Math.copySign(norm * (maxOutput - minOutput), rawValue) + minOutput;
        return norm;
    }


}
