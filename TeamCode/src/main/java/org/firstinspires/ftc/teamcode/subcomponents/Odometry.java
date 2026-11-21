package org.firstinspires.ftc.teamcode.subcomponents;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
}
