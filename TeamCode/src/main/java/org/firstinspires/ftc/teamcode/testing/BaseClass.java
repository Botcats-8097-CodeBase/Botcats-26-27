package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Disabled
@TeleOp (name = "", group = "Testing")
public class BaseClass extends LinearOpMode {
    List<LynxModule> allHubs;

    ElapsedTime elapsedTime = new ElapsedTime();
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry,
            PanelsTelemetry.INSTANCE.getFtcTelemetry());

    @Override
    public void runOpMode() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        waitForStart();
        elapsedTime.reset();

        while (opModeIsActive()) {

        }
    }
}
