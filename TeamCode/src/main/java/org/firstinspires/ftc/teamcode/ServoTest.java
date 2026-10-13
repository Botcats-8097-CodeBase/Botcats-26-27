package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "test servo")
public class ServoTest extends LinearOpMode {

    Servo servo = null;
    double pos = 0;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo1");
        servo.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(pos);

            if (gamepad1.y) {
                pos += 0.01;
            }
            if (gamepad1.a) {
                pos -= 0.01;
            }

            if (pos > 1) {
                pos = 1;
            }
            if (pos < -1) {
                pos = -1;
            }

            telemetry.addData("servo pos", pos);
            telemetry.update();
        }
    }
}
