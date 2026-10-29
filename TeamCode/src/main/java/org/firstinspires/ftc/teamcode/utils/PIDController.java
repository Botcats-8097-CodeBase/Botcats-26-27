package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    //PID coefficients
    double Kp, Ki, Kd, maxIntegral = 0;
    double lastError, integralSum = 0;
    double output = 0;
    boolean initialized = false;

    //set object attributes to instance
    public PIDController(double Kp, double Ki, double Kd, double maxIntegral) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.maxIntegral = Math.abs(maxIntegral);
    }

    //performs PID math, returns desired number
    public double update(double error, double dt) {
        if (dt <= 0) return output;
        if (!initialized) {
            lastError = error;
            initialized = true;
        }

        if (Ki != 0) {
            integralSum += error * dt;
            integralSum = Math.max(
                    -maxIntegral,
                    Math.min(maxIntegral, integralSum)
            );
        }
        double derivative = (error - lastError) / dt;

        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
        return output;
    }

    public double output() {
        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        output = 0;
        initialized = false;
    }
}
