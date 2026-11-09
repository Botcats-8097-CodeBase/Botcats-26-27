package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

public class Intake {
    private DcMotor intakeMotor;
    private double ticksPerRev;

    public Intake() {}

    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.intake);
        intakeMotor.setDirection(RobotConstants.intakeDirection);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ticksPerRev = intakeMotor.getMotorType().getTicksPerRev();
    }
    public void setIntakeMotor(double speed){
        double power = speed;
        intakeMotor.setPower(power);
    }

     public double getIntakeRevs(){
        return intakeMotor.getCurrentPosition() / ticksPerRev * 2;
     }
}
