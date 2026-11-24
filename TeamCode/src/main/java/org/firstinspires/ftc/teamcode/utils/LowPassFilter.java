package org.firstinspires.ftc.teamcode.utils;

public class LowPassFilter {
    private final double alpha;
    private double lastValue = 0.0;
    private boolean initialized = false;

    //Constructor; validates alpha
    public LowPassFilter(double alpha) {
        if (!Double.isFinite(alpha) || alpha < 0 || alpha > 1) {
            throw new IllegalArgumentException("Alpha must be between 0 and 1");
        }
        this.alpha = alpha;
    }

    //Filters new data with old data to smooth
    public void update(double rawValue) {
        if (!initialized) {
            lastValue = rawValue;
            initialized = true;
        }
        else {
            lastValue = (alpha * rawValue) + ((1.0 - alpha) * lastValue);
        }
    }

    public double getValue() {
        return lastValue;
    }

    public void reset() {
        lastValue = 0.0;
        initialized = false;
    }
}
