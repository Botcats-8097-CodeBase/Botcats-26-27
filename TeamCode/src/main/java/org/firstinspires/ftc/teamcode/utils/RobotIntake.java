package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotIntake {
    private DcMotor intakeMotor;
    private double ticksPerRev;

    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRev = intakeMotor.getMotorType().getTicksPerRev();

    }
    public void setIntakeMotor(double speed){
        intakeMotor.setPower(speed);

    }
     public double getIntakeRevs(){
        return intakeMotor.getCurrentPosition()/ ticksPerRev * 2;
     }
}
