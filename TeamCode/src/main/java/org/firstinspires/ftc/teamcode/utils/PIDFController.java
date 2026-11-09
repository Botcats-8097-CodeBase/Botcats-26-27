package org.firstinspires.ftc.teamcode.utils;

public class PIDFController {
    //PIDF coefficients
    private final double kF;
    private double setpoint = 0;
    private final PIDController pid;

    //set object attributes to instance
    public PIDFController(double Kp, double Ki, double Kd, double kF, double maxIntegral) {
        pid = new PIDController(Kp, Ki, Kd, maxIntegral);
        this.kF = kF;
    }

    // feedforward + PID
    public double update(double measurement, double dt) {
        double error = setpoint - measurement;
        return kF * setpoint + pid.update(error, dt);
    }

    public void setSetpoint(double target) {
        setpoint = target;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void reset() {
        pid.reset();
    }
}
