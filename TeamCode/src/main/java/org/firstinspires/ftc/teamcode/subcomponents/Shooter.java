package org.firstinspires.ftc.teamcode.subcomponents;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.TylerMath;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;

import javax.xml.transform.dom.DOMLocator;

public class Shooter {

    public enum State {
        IDLE, SPINNING_UP, READY, FIRING
    }
    public enum Mode {
        READY_SHOOT, HOLD_TO_SPIN
    }

    private State state = State.IDLE;
    private Mode mode = Mode.HOLD_TO_SPIN;

    private ElapsedTime dt = new ElapsedTime();

    private final PIDController turnToPID = RobotConstants.turnToGoalController;
    private final VelocityMotor flywheel;
    private final Blocker blocker;
    private final Intake intake;
    private final Limelight limelight;
    private final Odometry odometry;

    private final Pose redGoal = new Pose(60, 72);
    private final Pose blueGoal = new Pose(84, 72);

    private double targVel = 0;
    private double camError;

    private int targ1;
    private int targ2;

    public boolean fireRequest = false;
    private boolean intakeRequest = false;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    private Direction direction = Direction.FORWARD;

    private static final double INTAKE_POWER = 1.0;

    public Shooter(VelocityMotor flywheel, Blocker blocker, Intake intake, double camError, Odometry odometry, Limelight limelight) {
        this.flywheel = flywheel;
        this.blocker = blocker;
        this.intake = intake;
        this.camError = camError;
        this.odometry = odometry;
        this.limelight = limelight;
    }

    public void init(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction, int targ1, int targ2) {
        flywheel.init(hardwareMap, name, direction);
        blocker.init(hardwareMap);
        intake.init(hardwareMap);
        limelight.init(hardwareMap);
        this.targ1 = targ1;
        this.targ2 = targ2;
    }

    public void setTargets(int targ1, int targ2) {
        this.targ1 = targ1;
        this.targ2 = targ2;
    }

    public double goalDist(boolean isRed) {
        if (isRed) return odometry.distanceTo(redGoal);
        return odometry.distanceTo(blueGoal);
    }

    public void spinUp(double velocity) {
        if (velocity <= 0) {
            stop();
            return;
        }

        if (state == State.IDLE || velocity != targVel) {
            targVel = velocity;
            flywheel.setTargetVelocity(velocity);
            state = State.SPINNING_UP;
        }
    }

    public void setFire(boolean firing) {
        fireRequest = firing;
    }

    public void setIntake(boolean button1, boolean button2) {
        intakeRequest = button1 || button2;
        if (button1) {
            direction = Direction.FORWARD;
        }
        else if (button2) {
            direction = Direction.REVERSE;
        }
        else {
            direction = Direction.FORWARD;
        }
    }

    public void stop() {
        targVel = 0;
        fireRequest = false;
        intakeRequest = false;

        blocker.setPosition(RobotConstants.BLOCKER_CLOSED);
        intake.setIntakeMotor(0);
        flywheel.stop();
        state = State.IDLE;
    }

    public void stopFlywheel() {
        targVel = 0;
        fireRequest = false;

        blocker.setPosition(RobotConstants.BLOCKER_CLOSED);
        flywheel.stop();
        state = State.IDLE;
    }

    public void control(double velocity, boolean aHeld) {
        setFire(aHeld);

        if (mode == Mode.READY_SHOOT) {
            spinUp(velocity);
        }
        else {
            if (aHeld) {
                spinUp(velocity);

            }
            else if (state != State.IDLE) {
                stopFlywheel();
            }
        }
    }

    public double aim(boolean closeOrFar) {
        double targX = getTx(closeOrFar);

        if (!Double.isFinite(targX)) {
            resetAim();
            return 0;
        }

        double elapsed = Math.max(dt.seconds(), 0.001);
        dt.reset();

        return TylerMath.clamp(turnToPID.update(targX, elapsed), -1, 1);
    }

    public boolean shouldAim() {
        return fireRequest &&
                (state == State.SPINNING_UP ||
                        state == State.READY ||
                        state == State.FIRING);
    }

    public void resetAim() {
        turnToPID.reset();
        dt.reset();
    }

    public void update() {
        switch (state) {
            case IDLE:
                flywheel.update();
                break;

            case SPINNING_UP:
                flywheel.update();

                // && aimedAtTarg(odometry.side(), camError)
                if (atTargVel()) {

                    state = State.READY;
                }
                break;

            case READY:
                flywheel.update();

                if (!atTargVel()) {
                    state = State.SPINNING_UP;
                }
                else if (fireRequest) {
                    state = State.FIRING;
                }
                break;

            case FIRING:
                flywheel.update();

                if (!fireRequest) {
                    state = atTargVel() ? State.READY : State.SPINNING_UP;
                }
                break;
        }

        boolean firing = state == State.FIRING;

        blocker.setPosition(firing ? RobotConstants.BLOCKER_OPEN : RobotConstants.BLOCKER_CLOSED);
        intake.setIntakeMotor(
                (intakeRequest || firing)
                ? (direction == Direction.FORWARD ? INTAKE_POWER : -INTAKE_POWER)
                        : 0
        );
    }

    public void toggleMode(boolean wasPressed) {
        if (wasPressed) {
            if (mode == Mode.READY_SHOOT) {
                mode = Mode.HOLD_TO_SPIN;
            }
            else {
                mode = Mode.READY_SHOOT;
            }
        }
    }

    private boolean atTargVel() {
        double errorBounds = 200;
        return Math.abs(flywheel.getCurrVel() - targVel) <= errorBounds;
    }

    private boolean aimedAtTarg(boolean closeOrFar, double errorBounds) {
        if (!closeOrFar) {
            return Math.abs(limelight.limeScanPosX(targ1)) <= errorBounds;
        }
        else if (closeOrFar) {
            return Math.abs(limelight.limeScanPosX(targ2)) <= errorBounds;
        }

        return false;
    }

    public State getState() {
        return state;
    }

    public Mode getMode() {
        return mode;
    }

    public double getVelocity() {
        return flywheel.getCurrVel();
    }

    public double getTargVel() {
        return flywheel.getTargetVelocity();
    }

    public double getAppliedPower() {
        return flywheel.getAppliedPower();
    }

    public double getTx(boolean closeOrFar) {
        if (!closeOrFar) {
            return limelight.limeScanPosX(targ1);
        } else if (closeOrFar) {
            return limelight.limeScanPosX(targ2);
        }
        return Double.NaN;
    }
}
