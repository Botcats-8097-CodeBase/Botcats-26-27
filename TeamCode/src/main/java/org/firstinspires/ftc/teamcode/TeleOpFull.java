package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

import java.util.List;

@TeleOp (name = "TeleOpFull")
public class TeleOpFull extends OpMode {

    List<LynxModule> allHubs;

    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    ElapsedTime et = new ElapsedTime();

    BasicRobot robot = new BasicRobot();

    public IMU imu;

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );


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
            imu.resetYaw();
            imu.initialize(parameters);
        }

        //retrieve robot yaw and control robot w/ joystick
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        robot.drivePower(new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateVector(-Math.toRadians(yaw-yawOffset)), gamepad1.right_stick_x, coefficient);

    }

    void initRobot() {
        //hardware mapping expansion hub, imu
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        imu = hardwareMap.get(IMU.class, "imu");

        //class init
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        //blackboard + side switching
    }
}
