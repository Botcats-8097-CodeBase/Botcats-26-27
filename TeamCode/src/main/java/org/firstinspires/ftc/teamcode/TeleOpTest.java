package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.BasicRobot;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

import java.util.List;

@TeleOp (name = "TeleOpTest")
public class TeleOpTest extends OpMode {

    List<LynxModule> allHubs;

    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    ElapsedTime et = new ElapsedTime();

    BasicRobot robot = new BasicRobot();
    VelocityMotor shooter = new VelocityMotor(
            new PIDFController(0.001, 0, 0, 0.000397, 0),
            0.3);

    Intake intake = new Intake();

    public IMU imu;

    IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );


    double yawOffset = 90;
    double coefficient;

    double intakeSpeed = 0;
    double targVel = 2500;

    boolean wasShooting = false;

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

        if (gamepad1.dpadUpWasPressed()) {
            intakeSpeed += 0.1;
        }
        if (gamepad1.dpadDownWasPressed()) {
            intakeSpeed -= 0.1;
        }
        if (gamepad1.right_bumper) {
            intake.setIntakeMotor(intakeSpeed);
        }
        else {
            intake.setIntakeMotor(0);
        }

        if (gamepad1.a) {
            shooter.setTargetVelocity(targVel);
        }
        else if (wasShooting) {
            shooter.stop();
        }
        wasShooting = gamepad1.a;
        shooter.update();

        pTelemetry.addData("targVel", shooter.getTargetVelocity());
        pTelemetry.addData("currentVel", shooter.getCurrVel());
        pTelemetry.addData("intakeSpeed", intakeSpeed);
        pTelemetry.addData("shooterPower", shooter.getAppliedPower());

        pTelemetry.update();
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
        intake.init(hardwareMap);
        shooter.init(hardwareMap, RobotConstants.pollenSpinnerName, RobotConstants.pollenSpinnerDirection);
    }

    @Override
    public void init_loop() {
        //blackboard + side switching
    }
}
