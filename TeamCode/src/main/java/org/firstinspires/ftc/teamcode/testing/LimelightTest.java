package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subcomponents.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@Disabled
@TeleOp (name = "Limelight Test", group = "Testing")
public class LimelightTest extends LinearOpMode {
    List<LynxModule> allHubs;

    ElapsedTime elapsedTime = new ElapsedTime();
    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry,
            PanelsTelemetry.INSTANCE.getFtcTelemetry());
    BasicRobot robot = new BasicRobot();

    PIDController pidController = new PIDController(0.001, 0.0, 0.0, 0.0);
    Limelight limelight = new Limelight(false, 0.0, 0.0, 0.0);

    public IMU imu;

    double yawOffset = 90;
    double coefficient;

    @Override
    public void runOpMode() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");


        waitForStart();
        elapsedTime.reset();

        while (opModeIsActive()) {
            double dt = elapsedTime.seconds();
            elapsedTime.reset();
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            robot.drivePower(
                    new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateVector(-Math.toRadians(yaw - yawOffset)),
                    Math.max(-1, Math.min(1, pidController.update(limelight.limeScanPosX(1), dt))),
                    coefficient);

        }
    }
}
