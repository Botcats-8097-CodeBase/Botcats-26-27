
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Intake;

@TeleOp (name = "Intake Motor Test")
public class IntakeMotorTest extends OpMode {
    Intake intake = new Intake();

    @Override
    public void init() {
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        intake.setIntakeMotor(1);
        telemetry.addData("Intake Revs", intake.getIntakeRevs());
    }
}
