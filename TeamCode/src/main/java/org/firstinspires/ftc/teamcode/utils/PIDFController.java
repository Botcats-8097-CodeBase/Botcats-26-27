package org.firstinspires.ftc.teamcode.utils;

public class PIDFController {
    //PIDF coefficients
    double Kv;
    double output = 0;
    boolean initialized = false;
    double targetVel = 0;
    PIDController velocityPID;



    //set object attributes to instance
    public PIDFController(double Kp, double Ki, double Kd, double Kv, double maxIntegral) {
        velocityPID = new PIDController(Kp, Ki, Kd, maxIntegral);
        this.Kv = Kv;
    }

    //todo: use velocity motor class to calculate velocity error with data filter
    public double update(double error, double dt) {
//        if (dt <= 0) return output;
//        if (!initialized) {
//            initialized = true;
//        }

        double correction = velocityPID.update(error, dt);

        // feedforward + PID
        double output = Kv * targetVel + correction;
        return output;
    }

    public void setTargetVelocity(double target) {
        targetVel = target;
    }

    public double getTargetVelocity() {
        return targetVel;
    }

    public void reset() {
        output = 0;
        initialized = false;
    }
}
