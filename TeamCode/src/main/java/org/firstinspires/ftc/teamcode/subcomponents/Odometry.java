package org.firstinspires.ftc.teamcode.subcomponents;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Odometry {
    private Follower follower;

    public void init(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public double getYaw() {
        follower.update();
        return Math.toDegrees(follower.pose().heading());
    }

    public void resetHeading(double heading) {
        follower.setHeading(heading);
    }

    public double distanceTo(Pose pose) {
        follower.update();
        return follower.pose().distance(pose);
    }

    public boolean side() {
        follower.update();
        return follower.pose().y() > 72;
    }

    public Pose getPose() {
        follower.update();
        return follower.pose();
    }

    public void setPose(Pose2D pose) {
        follower.setPose(new Pose(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH)));
        follower.setHeading(pose.getHeading(AngleUnit.RADIANS));
    }

    public void updateControl(double leftStickX, double leftStickY, double rightStickX) {
        ManualDrive.driveOrHold(
                follower,
                ManualDrive.fieldCentric(
                        -leftStickY,
                        leftStickX,
                        rightStickX,
                        follower.pose().heading()
                )
        );

        follower.update();
    }
}
