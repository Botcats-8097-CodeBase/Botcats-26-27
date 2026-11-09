package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.PIDFController;

@Disabled
public class PIDFTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        PIDFController controller = new PIDFController(0.001, 0, 0, 0.000397, 0);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
