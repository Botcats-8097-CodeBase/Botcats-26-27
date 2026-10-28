package org.firstinspires.ftc.teamcode.utils;

public class LowPassFilter {
    private double alpha;
    private double lastValue = 0.0;
    private boolean initialized = false;

    public LowPassFilter(double alpha) {
        this.alpha = alpha;
    }

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
}
