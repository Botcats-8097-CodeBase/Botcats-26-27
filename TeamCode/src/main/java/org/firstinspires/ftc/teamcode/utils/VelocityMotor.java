package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityMotor {
    private double lastPos = 0;
    private double actlVel = 0;

    private double appliedPower = 0;

    private DcMotor motor;
    private final PIDFController vController;
    private final LowPassFilter velFilter;
    private final ElapsedTime elapsedTime = new ElapsedTime();

    //Constructor
    public VelocityMotor(PIDFController controller, double alpha) {
        this.vController = controller;
        this.velFilter = new LowPassFilter(alpha);
    }



    //init protocol
    public void init(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(direction);

        reset();
    }

    //update motor
    public void update() {
        double dt = elapsedTime.seconds();
        if (dt < 0.001) return;
        elapsedTime.reset();

        double currPos = motor.getCurrentPosition();

        double rawVel = (currPos - lastPos) / dt;
        lastPos = currPos;
        velFilter.update(rawVel);

        actlVel = velFilter.getValue();

        if (vController.getSetpoint() == 0) {
            appliedPower = 0;
            motor.setPower(0);
            return;
        }

        double power = vController.update(actlVel, dt);

        if (!Double.isFinite(power)) {
            stop();
            return;
        }

        power = Math.max(-1, Math.min(1, power));

        appliedPower = power;
        motor.setPower(power);
    }

    public void setTargetVelocity(double target) {
        vController.setSetpoint(target);
    }

    public double getTargetVelocity() {
        return vController.getSetpoint();
    }

    public double getAppliedPower() {
        return appliedPower;
    }

    public double getCurrVel() {
        return actlVel;
    }

    public void reset() {
        motor.setPower(0);

        vController.setSetpoint(0);
        vController.reset();
        velFilter.reset();

        actlVel = 0;
        appliedPower = 0;
        lastPos = motor.getCurrentPosition();
        elapsedTime.reset();
    }

    public void stop() {
        vController.setSetpoint(0);
        vController.reset();
        appliedPower = 0;

        motor.setPower(0);
    }
}
