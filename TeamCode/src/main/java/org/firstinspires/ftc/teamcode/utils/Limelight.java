package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class Limelight {
    private Limelight3A limelight;
    private boolean hasTarget;
    private double tx;       // Horizontal offset (-29.8 to 29.8 deg)
    private double ty;       // Vertical offset (-24.8 to 24.8 deg)
    private double ta;       // Target area (0% to 100% of image)


    // A modern Java record to hold all the coordinate data neatly
    public Limelight(boolean hasTarget, double tx, double ty, double ta) {
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

    public Limelight getTargetData(Limelight3A limelightInstance) {
        if (limelightInstance == null) {
            return new Limelight(false, 0.0, 0.0, 0.0);
        }

        LLResult result = limelightInstance.getLatestResult();

        if (result == null || !result.isValid()) {
            return new Limelight(false, 0.0, 0.0, 0.0);
        }

        return new Limelight(true, result.getTx(), result.getTy(), result.getTa());

    }

    public double limeScanPosX(int tagID) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducialResults) {
                if (f.getFiducialId() == tagID) {
                    return f.getTargetXDegrees();
                }
            }
        }
        return Double.NaN;
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