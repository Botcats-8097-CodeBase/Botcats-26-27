package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Day 1")
public class Day1 extends LinearOpMode {

    DcMotor spinner;

    @Override
    public void runOpMode() {
        // telemetry
        spinner = hardwareMap.get(DcMotor.class, "spinnerMotor");
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        double speed = 0;

        waitForStart();
        while (opModeIsActive()) {
            //button feedback: telemetry statement
            if (gamepad1.aWasPressed()) {
                speed += 0.1;
            }
            if (gamepad1.yWasPressed()) {
                speed -= 0.1;
            }
            spinner.setPower(speed);

        }
    }
}
