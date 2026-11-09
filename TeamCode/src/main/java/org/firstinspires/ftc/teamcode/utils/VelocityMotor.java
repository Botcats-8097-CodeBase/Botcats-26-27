package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityMotor {
    double lastPos = 0;
    double alpha;
    double actlVel = 0;
    double targetVelocity;
    PIDFController vController = new PIDFController(0.001, 0, 0, 0, 0);

    public VelocityMotor(double alpha) {
        this.alpha = alpha;
    }

    public void setVelController(PIDFController controller) {
        vController = controller;
    }

    public DcMotor motor;
    LowPassFilter velFilter = new LowPassFilter(alpha);
    ElapsedTime elapsedTime = new ElapsedTime();

    public void init(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        double dt = elapsedTime.seconds();
        elapsedTime.reset();

        double currPos = motor.getCurrentPosition();

        double rawVel = (currPos - lastPos) / dt;
        lastPos = currPos;
        velFilter.update(rawVel);

        actlVel = velFilter.getValue();

        double errorVel = vController.getTargetVelocity() - actlVel;

        double power = vController.update(errorVel, dt);
        power = Math.max(-1, Math.min(1, power));

        motor.setPower(power);
    }

    public void setTargetVelocity(double target) {
        vController.setTargetVelocity(target);
        targetVelocity = target;
    }

    public double getTargetVelocity() {
        return vController.getTargetVelocity();
    }

    public double getCurrVel() {
        return actlVel;
    }
}
