package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotConstants {
    public static String leftBackDriveName = "backLeft";
    public static String rightBackDriveName = "backRight";
    public static String leftFrontDriveName = "frontLeft";
    public static String rightFrontDriveName = "frontRight";

    public static DcMotorSimple.Direction leftBackDriveDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightBackDriveDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftFrontDriveDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontDriveDirection = DcMotorSimple.Direction.FORWARD;

    //---------//

    public static String pollenSpinnerName = "pollenSpinner";
    public static DcMotorSimple.Direction pollenSpinnerDirection = DcMotorSimple.Direction.FORWARD;

    public static PIDFController tunedController = new PIDFController(0.001, 0, 0, 0.000397, 0);
    public static double dataFilterValue = 0.3;

    //---------//

    public static String intakeMotorName = "intakeMotor";
    public static DcMotorSimple.Direction intakeDirection = DcMotorSimple.Direction.REVERSE;

    public static String blockerName = "blockerName";
    public static double BLOCKER_OPEN = 0;
    public static double BLOCKER_CLOSED = 1;
}
