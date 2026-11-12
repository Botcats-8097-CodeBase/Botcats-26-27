package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subcomponents.Blocker;

import java.util.List;

@TeleOp (name = "blockerTest", group = "Testing")
public class BlockerTest extends LinearOpMode {
    List<LynxModule> allHubs;

    ElapsedTime elapsedTime = new ElapsedTime();
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry,
            PanelsTelemetry.INSTANCE.getFtcTelemetry());

    Blocker blocker = new Blocker();

    double servoPos = 0;

    @Override
    public void runOpMode() {

        blocker.init(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();
        elapsedTime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servoPos += 0.01;
            }
            if (gamepad1.b) {
                servoPos -= 0.01;
            }

            if (servoPos < 0) {
                servoPos = 0;
            }
            if (servoPos > 1) {
                servoPos = 1;
            }
            blocker.setPosition(servoPos);
        }
    }
}
