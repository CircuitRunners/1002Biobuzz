package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightCamera {
    private Limelight3A limelight;

    public LimelightCamera(HardwareMap hwmap) {
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public double [] getPollenDetectorResults() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        LLResultTypes.DetectorResult best = null;
        for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
            if (dr == null || dr.getTargetArea() < 0.005) continue;
            if (best == null || dr.getTargetArea() > best.getTargetArea()) best = dr;
        }
        if (best == null) return null;
        return new double[]{best.getTargetXDegrees(), best.getTargetYDegrees(), best.getTargetArea()};
    }

}
