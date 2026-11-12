package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subcomponents.Blocker;
import org.firstinspires.ftc.teamcode.subcomponents.Intake;
import org.firstinspires.ftc.teamcode.subcomponents.Shooter;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

import java.util.List;

@TeleOp (name = "shooterTest", group = "Testing")
public class ShooterTest extends LinearOpMode {

    List<LynxModule> allHubs;

    ElapsedTime elapsedTime = new ElapsedTime();
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry,
            PanelsTelemetry.INSTANCE.getFtcTelemetry());

    Shooter shooter = new Shooter(
            new VelocityMotor(
                    RobotConstants.tunedController,
                    RobotConstants.dataFilterValue
            ),
            new Blocker(),
            new Intake()
    );

    @Override
    public void runOpMode() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        shooter.init(
                hardwareMap,
                RobotConstants.pollenSpinnerName,
                RobotConstants.pollenSpinnerDirection
        );

        waitForStart();
        elapsedTime.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double dt = elapsedTime.seconds();
            elapsedTime.reset();

            if (gamepad1.backWasPressed()) {
                shooter.toggleMode();
            }

            shooter.control(2500, gamepad1.a);
            shooter.setIntake(gamepad1.right_bumper);
            shooter.update();

            pTelemetry.addData("Mode", shooter.getMode());
            pTelemetry.addData("State", shooter.getState());
            pTelemetry.update();
        }
    }
}
