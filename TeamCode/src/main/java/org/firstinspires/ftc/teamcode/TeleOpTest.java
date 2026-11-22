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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subcomponents.BasicRobot;
import org.firstinspires.ftc.teamcode.subcomponents.Blocker;
import org.firstinspires.ftc.teamcode.subcomponents.Intake;
import org.firstinspires.ftc.teamcode.subcomponents.Odometry;
import org.firstinspires.ftc.teamcode.subcomponents.Shooter;
import org.firstinspires.ftc.teamcode.utils.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.TylerMath;
import org.firstinspires.ftc.teamcode.utils.Vector2D;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

import java.util.List;

@TeleOp (name = "TeleOpTest", group = "Testing")
public class TeleOpTest extends OpMode {

    List<LynxModule> allHubs;

    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    ElapsedTime et = new ElapsedTime();
    Pose2D initPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    Odometry odometry = new Odometry();
    Limelight limelight = new Limelight();

    BasicRobot robot = new BasicRobot();
    Shooter shooter = new Shooter(
            new VelocityMotor(RobotConstants.tunedController, RobotConstants.dataFilterValue),
            new Blocker(),
            new Intake(),
            1.5,
            odometry,
            limelight
    );

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
    double targVel = 2850;

    int targ1;
    int targ2;

    String[] color = {"blue", "red"};

    boolean isBlackBoardPos = true;
    boolean isRed = false;
    boolean isClose =true;


    public enum YawTrack {
        ODO, IMU
    }

    YawTrack yawTrack = YawTrack.ODO;

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
        et.reset();



        if (isBlackBoardPos && blackboard.get("x") != null && blackboard.get("y") != null && blackboard.get("heading") != null && blackboard.get("yawPos") != null) {
            initPose = new Pose2D(
                    DistanceUnit.INCH,
                    (double) blackboard.get("x"),
                    (double) blackboard.get("y"),
                    AngleUnit.DEGREES,
                    TylerMath.wrap(((double) blackboard.get("heading")), -180, 180));
        } else {
            if (!isRed) {
                if (isClose) initPose = new Pose2D(DistanceUnit.INCH, 84, 7.5, AngleUnit.DEGREES, 90);
                else initPose = new Pose2D(DistanceUnit.INCH, 89.5, 136.5, AngleUnit.DEGREES, 270);


            } else {
                //18/2,15.5/2,0
                if (isClose) initPose = new Pose2D(DistanceUnit.INCH, 54.5, 7.5, AngleUnit.DEGREES, 90);
                else initPose = new Pose2D(DistanceUnit.INCH, 60, 136.5, AngleUnit.DEGREES, 270);
            }
        }

        targ1 = isRed ? 35 : 40;
        targ2 = isRed ? 32 : 43;
        shooter.setTargets(targ1, targ2);
        odometry.setPose(initPose);

        if (!isRed) {
            yawOffset = -180;
        } else {
            yawOffset = 0;
        }
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

            imu.resetYaw();
            imu.initialize(parameters);
        }


        double yaw;
        //retrieve robot yaw and control robot w/ joysticks
        if (yawTrack == YawTrack.IMU) {
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        }
        else {
            yaw = odometry.getYaw();
        }

        Vector2D move = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateVector(-Math.toRadians(yaw - yawOffset));
        //set shooter
        shooter.toggleMode(gamepad1.backWasPressed());
        shooter.control(2450, gamepad1.a);

        double rotation = gamepad1.right_stick_x;

        if (shooter.shouldAim()) {
            rotation = shooter.aim(odometry.side());
        }
        else {
            shooter.resetAim();
        }

        shooter.setIntake(gamepad1.right_bumper, gamepad1.left_bumper);
        shooter.update();

        robot.drivePower(move, rotation, coefficient);

        pTelemetry.addData("goalDist", shooter.goalDist(isRed));
        pTelemetry.addData("x", odometry.getPose().x());
        pTelemetry.addData("y", odometry.getPose().y());
        pTelemetry.addData("yaw mode", yawTrack);
        pTelemetry.addData("yaw", yaw);
        pTelemetry.addData("shooter mode", shooter.getMode());
        pTelemetry.addData("shooter state", shooter.getState());
        pTelemetry.addData("targVel", shooter.getTargVel());
        pTelemetry.addData("currentVel", shooter.getVelocity());
        pTelemetry.addData("targClose", targ1);
        pTelemetry.addData("targFar", targ2);
        pTelemetry.addData("tx", shooter.getTx(odometry.side()));
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
        odometry.init(hardwareMap);

        //class init
        robot.init(hardwareMap);
        shooter.init(
                hardwareMap,
                RobotConstants.pollenSpinnerName,
                RobotConstants.pollenSpinnerDirection,
                targ1, targ2
        );
    }

    @Override
    public void init_loop() {
        //blackboard + side switching
        int colorNum = isRed ? 1 : 0;


        pTelemetry.addData("team", color[colorNum]);
        pTelemetry.addData("is close", isClose);
        pTelemetry.addData("is black board pos", isBlackBoardPos);
        pTelemetry.addData("x", blackboard.get("x"));
        pTelemetry.addData("y", blackboard.get("y"));
        pTelemetry.addData("heading", blackboard.get("heading"));
        pTelemetry.addData("turret heading", blackboard.get("yawPos"));

        if (gamepad1.aWasPressed()) isRed = !isRed;
        if (gamepad1.bWasPressed()) isClose = !isClose;
        if (gamepad1.xWasPressed()) isBlackBoardPos = !isBlackBoardPos;

        pTelemetry.update();
    }
}
