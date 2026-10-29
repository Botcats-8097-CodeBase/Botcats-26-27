package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LimelightCameraThing {
    private Limelight3A limelight;
    public boolean hasTarget;
    public double tx;       // Horizontal offset (-29.8 to 29.8 deg)
    public double ty;       // Vertical offset (-24.8 to 24.8 deg)
    public double ta;       // Target area (0% to 100% of image)




    // A modern Java record to hold all the coordinate data neatly
    public LimelightCameraThing(boolean hasTarget, double tx, double ty, double ta) {
        this.hasTarget = hasTarget;
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
    }

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public static LimelightCameraThing getTargetData(Limelight3A limelightInstance) {
        if (limelightInstance == null) {
            return new LimelightCameraThing(false, 0.0, 0.0, 0.0);
        }
        LLResult result = limelightInstance.getLatestResult();
        if (result == null || !result.isValid()) {
            return new LimelightCameraThing(false, 0.0, 0.0, 0.0);
        }

        return new LimelightCameraThing(true, result.getTx(), result.getTy(), result.getTa());
    }
    public Vector2D toVector() {
        return new Vector2D(this.tx, this.ty);
    }
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
