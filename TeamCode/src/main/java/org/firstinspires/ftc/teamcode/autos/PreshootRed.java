
package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subcomponents.Blocker;
import org.firstinspires.ftc.teamcode.subcomponents.Intake;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

@Autonomous(name = "Preshoot Red", group = "Autonomous")
public class PreshootRed extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(54.5, 7.5, 90);

    private final Pose path1 =
            poseFactory.of(12.1395, 95.6939 - 4, 0);

    private final Pose path1Control1 =
            poseFactory.of(2.5663, 14.9063, 0);

    private final Pose path1Control2 =
            poseFactory.of(32.5866, 85.8278, 0);

    // Shooter hardware
    private final VelocityMotor flywheel = new VelocityMotor(
            RobotConstants.tunedController,
            RobotConstants.dataFilterValue
    );

    private final Blocker blocker = new Blocker();
    private final Intake intake = new Intake();

    // Shooting settings
    private static final double TARGET_VELOCITY = 2450;
    private static final double VELOCITY_TOLERANCE = 200;

    private static final double FEED_TIME = 3.0;
    private static final double SPINUP_TIMEOUT = 5.0;
    private static final double INTAKE_POWER = 1.0;

    private final ElapsedTime timer = new ElapsedTime();

    // Autonomous states
    private enum State {
        SPINNING_UP,
        FIRING,
        PARKING,
        DONE
    }

    private State state = State.SPINNING_UP;
    private boolean parkingStarted = false;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        // Initialize PedroPathing
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        // Initialize shooter hardware
        flywheel.init(
                hardwareMap,
                RobotConstants.pollenSpinnerName,
                RobotConstants.pollenSpinnerDirection
        );

        blocker.init(hardwareMap);
        intake.init(hardwareMap);

        // Safe starting positions
        blocker.setPosition(RobotConstants.BLOCKER_CLOSED);
        intake.setIntakeMotor(0);
        flywheel.stop();

        telemetry.addLine("Preshoot Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopShooter();
            return;
        }

        state = State.SPINNING_UP;
        timer.reset();

        flywheel.setTargetVelocity(TARGET_VELOCITY);

        try {
            while (opModeIsActive() && state != State.DONE) {

                // Keep velocity PIDF updating during shooting
                if (state == State.SPINNING_UP ||
                        state == State.FIRING) {
                    flywheel.update();
                }

                switch (state) {

                    case SPINNING_UP:

                        // Wait for flywheel to reach target speed
                        if (atTargetVelocity()) {

                            blocker.setPosition(
                                    RobotConstants.BLOCKER_OPEN
                            );

                            intake.setIntakeMotor(INTAKE_POWER);

                            timer.reset();
                            state = State.FIRING;
                        }

                        // Prevent getting stuck if velocity
                        // cannot be reached
                        else if (timer.seconds() >= SPINUP_TIMEOUT) {

                            stopShooter();
                            state = State.PARKING;
                        }

                        break;

                    case FIRING:

                        // Feed artifacts for a fixed duration
                        if (timer.seconds() >= FEED_TIME) {

                            stopShooter();
                            state = State.PARKING;
                        }

                        break;

                    case PARKING:

                        if (!parkingStarted) {
                            schedule(follow(follower, path1()));
                            parkingStarted = true;
                        }

                        follower.update();
                        Scheduler.execute();

                        savePoseToBlackboard();

                        if (parkingStarted && !follower.isBusy()) {
                            state = State.DONE;
                        }

                        break;

                    case DONE:
                        break;
                }

                // Telemetry
                telemetry.addData("Auto State", state);
                telemetry.addData("Target Velocity", TARGET_VELOCITY);
                telemetry.addData("Current Velocity", flywheel.getCurrVel());
                telemetry.addData("Flywheel Power", flywheel.getAppliedPower());
                telemetry.addData("Timer", timer.seconds());

                telemetry.addData("X", follower.pose().x());
                telemetry.addData("Y", follower.pose().y());
                telemetry.addData("Heading", follower.pose().heading());

                telemetry.update();

                idle();
            }
        } finally {
            stopShooter();
            savePoseToBlackboard();
        }
    }

    // Check flywheel velocity
    private boolean atTargetVelocity() {
        return Math.abs(
                flywheel.getCurrVel() - TARGET_VELOCITY
        ) <= VELOCITY_TOLERANCE;
    }

    // Stop all shooting mechanisms
    private void stopShooter() {
        blocker.setPosition(RobotConstants.BLOCKER_CLOSED);
        intake.setIntakeMotor(0);
        flywheel.stop();
    }

    // Parking path
    public Path path1() {
        return Paths.curve(
                start,
                path1Control1,
                path1Control2,
                path1
        ).heading(Interpolator.constant(Math.toRadians(90)));
    }

    private void savePoseToBlackboard() {
        Pose pose = follower.pose();

        blackboard.put("x", pose.x());
        blackboard.put("y", pose.y());
        blackboard.put("heading", Math.toDegrees(pose.heading()));
        blackboard.put("yawPos", 0.0);
    }
}
