package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "smallShootertest")
public class smallShooterTest extends LinearOpMode {
    DcMotor smallShooter = null;
    double smallShooterSpeed;

    @Override
    public void runOpMode() {
        smallShooter = hardwareMap.get(DcMotor.class, RobotConstants.smallShooterName);
        smallShooter.setDirection(RobotConstants.smallShooterDir);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                smallShooterSpeed += 0.1;
            }

            if (gamepad1.bWasPressed()) {
                smallShooterSpeed -= 0.1;
            }

            if (gamepad1.xWasPressed()) {
                smallShooterSpeed = 0;
            }

            if (smallShooterSpeed > 1) {
                smallShooterSpeed = 1;
            }
            if (smallShooterSpeed < -1) {
                smallShooterSpeed = -1;
            }

            smallShooter.setPower(smallShooterSpeed);

            telemetry.addData("smallShooter power: ", smallShooterSpeed);
            telemetry.update();
        }
    }
}
