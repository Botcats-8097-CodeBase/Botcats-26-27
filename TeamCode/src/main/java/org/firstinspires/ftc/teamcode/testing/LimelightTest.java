package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subcomponents.BasicRobot;
import org.firstinspires.ftc.teamcode.subcomponents.Odometry;
import org.firstinspires.ftc.teamcode.utils.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.TylerMath;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp (name = "Limelight Test", group = "testing")
public class LimelightTest extends OpMode {
    List<LynxModule> allHubs;

    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    ElapsedTime et = new ElapsedTime();

    BasicRobot robot = new BasicRobot();

    Odometry odometry = new Odometry();

    Limelight limelight = new Limelight();

    PIDController pidController = new PIDController(0.045, 0, 0.0005, 0);


    double yawOffset = 90;
    double coefficient;

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
        et.reset();

    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        //speed coef.
        if (gamepad1.right_trigger_pressed) {
            coefficient = 0.35;
        }
        else {
            coefficient = 1;
        }

        //reset imu
        if (gamepad1.y) {
            odometry.resetHeading(0);
        }

        //retrieve robot yaw and control robot w/ joystick

        double yaw = odometry.getYaw();

        Vector2D move = new Vector2D(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y
        ).rotateVector(-Math.toRadians(yaw - yawOffset));

        double rotation = gamepad1.right_stick_x;

        if (gamepad1.a) {
            double targetX = limelight.limeScanPosX(40);

            if (Double.isFinite(targetX)) {
                double dt = et.seconds();
                et.reset();

                rotation = TylerMath.clamp(
                        pidController.update(targetX, Math.max(dt, 0.001)),
                        -1, 1
                );
            } else {
                pidController.reset();
                et.reset();
                rotation = 0;
            }
        } else {
            pidController.reset();
            et.reset();
        }

        robot.drivePower(move, rotation, coefficient);
    }

    void initRobot() {
        //hardware mapping expansion hub, imu
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //class init
        odometry.init(hardwareMap);
        robot.init(hardwareMap);
        limelight.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        //blackboard + side switching
    }
}
