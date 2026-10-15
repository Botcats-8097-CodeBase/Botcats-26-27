package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotConstants {
        public static String motor1Name = "motor1";
        public static DcMotorSimple.Direction motor1Dir = DcMotorSimple.Direction.FORWARD;

        // Mecanum drivetrain. Names must match the Driver Hub robot configuration.
        public static String frontLeftName = "frontLeft";
        public static String frontRightName = "frontRight";
        public static String backLeftName = "backLeft";
        public static String backRightName = "backRight";

        // Left side is mounted mirrored, so it is usually reversed. Flip these if a wheel spins the wrong way.
        public static DcMotorSimple.Direction frontLeftDir = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction frontRightDir = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction backLeftDir = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction backRightDir = DcMotorSimple.Direction.FORWARD;

        public static String smallShooterName = "smallShooter";

        public static DcMotorSimple.Direction smallShooterDir = DcMotorSimple.Direction.FORWARD;

    }
