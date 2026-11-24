package org.firstinspires.ftc.teamcode.subcomponents;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.RobotConstants;

public class Blocker {
    private Servo blockerServo;

    public void init(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, RobotConstants.blockerName);
    }

    public void setPosition(double position) {
        blockerServo.setPosition(position);
    }

    public double getPosition() {
        return blockerServo.getPosition();
    }
}
