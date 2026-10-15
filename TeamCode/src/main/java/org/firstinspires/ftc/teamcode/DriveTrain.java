package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "driveTrain")
public class DriveTrain extends LinearOpMode {
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;
    static final double TEST_POWER = 0.4;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.frontLeftName);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.frontRightName);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.backLeftName);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.backRightName);

        frontLeft.setDirection(RobotConstants.frontLeftDir);
        frontRight.setDirection(RobotConstants.frontRightDir);
        backLeft.setDirection(RobotConstants.backLeftDir);
        backRight.setDirection(RobotConstants.backRightDir);

        telemetry.addLine("Push the left stick to drive.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // The stick reports up as negative y, so flip it. Right is positive x.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            drive(y, x);

            telemetry.addData("stick y (forward): ", y);
            telemetry.addData("stick x (right): ", x);
            telemetry.update();
        }
        stopAll();
    }

    // y: 1 forward, -1 backward. x: 1 right, -1 left. Partial values give partial speed.
    void drive(double y, double x) {
        double fl = y + x;
        double fr = y - x;
        double bl = y - x;
        double br = y + x;

        // Diagonals can push a wheel past 1, so scale everything down together to keep the direction.
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        double scale = TEST_POWER / max;
        setPowers(fl * scale, fr * scale, bl * scale, br * scale);
    }

    void stopAll() {
        setPowers(0, 0, 0, 0);
    }

    void setPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
