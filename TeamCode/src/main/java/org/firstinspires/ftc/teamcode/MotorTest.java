package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "motorTest")
public class MotorTest extends LinearOpMode {
    DcMotor motor = null;
    double motorSpeed;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor1");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                motorSpeed += 0.1;
            }

            if (gamepad1.bWasPressed()) {
                motorSpeed -= 0.1;
            }

            if (gamepad1.xWasPressed()) {
                motorSpeed = 0;
            }

            if (motorSpeed > 1) {
                motorSpeed = 1;
            }
            if (motorSpeed < -1) {
                motorSpeed = -1;
            }

            motor.setPower(motorSpeed);

            telemetry.addData("motor power: ", motorSpeed);
            telemetry.update();
        }

    }
}
