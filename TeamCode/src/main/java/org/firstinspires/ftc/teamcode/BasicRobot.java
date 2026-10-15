package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BasicRobot {

    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    public BasicRobot() {}

    public void init(HardwareMap hardwareMap) {
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.leftBackDriveName);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.rightBackDriveName);
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.leftFrontDriveName);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.rightFrontDriveName);

        backLeft.setDirection(RobotConstants.leftBackDriveDirection);
        backRight.setDirection(RobotConstants.rightBackDriveDirection);
        frontLeft.setDirection(RobotConstants.leftFrontDriveDirection);
        frontRight.setDirection(RobotConstants.rightFrontDriveDirection);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drivePower(Vector2D moveVector, double rot, double speed) {
        double x = moveVector.x;
        double y = moveVector.y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1.1);
        double frontLeftPower = (y + x + rot) / denominator * speed;
        double backLeftPower = (y - x + rot) / denominator * speed;
        double frontRightPower = (y - x - rot) / denominator * speed;
        double backRightPower = (y + x - rot) / denominator * speed;

        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
    }
}
