package org.firstinspires.ftc.teamcode.subcomponents;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.VelocityMotor;

public class Shooter {

    public enum State {
        IDLE, SPINNING_UP, READY, FIRING
    }
    public enum Mode {
        READY_SHOOT, HOLD_TO_SPIN
    }

    private State state = State.IDLE;
    private Mode mode = Mode.HOLD_TO_SPIN;

    private final VelocityMotor flywheel;
    private final Blocker blocker;
    private final Intake intake;

    private double targVel = 0;

    private boolean fireRequest = false;
    private boolean intakeRequest = false;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    private Direction direction = Direction.FORWARD;

    private static final double INTAKE_POWER = 1.0;

    public Shooter(VelocityMotor flywheel, Blocker blocker, Intake intake) {
        this.flywheel = flywheel;
        this.blocker = blocker;
        this.intake = intake;
    }

    public void init(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        flywheel.init(hardwareMap, name, direction);
        blocker.init(hardwareMap);
        intake.init(hardwareMap);
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

    public void update() {
        switch (state) {
            case IDLE:
                flywheel.update();
                break;

            case SPINNING_UP:
                flywheel.update();

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
}
