package org.firstinspires.ftc.teamcode.subcomponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class Intake {
    private DcMotor intakeMotor;
    private double ticksPerRev;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.intakeMotorName);
        intakeMotor.setDirection(RobotConstants.intakeDirection);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ticksPerRev = intakeMotor.getMotorType().getTicksPerRev();
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.setPower(speed);
    }

     public double getIntakeRevs(){
        return intakeMotor.getCurrentPosition() / ticksPerRev * 2;
     }
}
