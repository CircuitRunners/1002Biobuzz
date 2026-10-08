package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Poses;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.EnumMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

public class SwyftCamera {
    public static long EXPOSURE_MS = 3;
    public static int GAIN = 250;
    private final VisionPortal portal;
    private final AprilTagProcessor aprilTag;
    private boolean liveStreamingEnabled = false;
    private boolean controlsApplied = false;
    private Set<Integer> validIDs = new LinkedHashSet<>();
    private static final int REQUIRED_IDS = 3;
    private final Map<Poses.Zone, Double> zoneLastTipMs = new EnumMap<>(Poses.Zone.class);
    private final ElapsedTime tipTimer = new ElapsedTime();
    private static final double TIP_CONFIRM_MS = 250;

    public SwyftCamera(HardwareMap hwmap) {
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true);

        aprilTag = tagBuilder.build();
        // 2 = faster, shorter range; 1 = slower, longer range
        aprilTag.setDecimation(2);

        portal = new VisionPortal.Builder()
                .setCamera(hwmap.get(WebcamName.class, "swyftCamera"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        for (Poses.Zone zone : Poses.Zone.values()) {
            zoneLastTipMs.put(zone, Double.NEGATIVE_INFINITY);
        }
    }

    /** Call every loop; applies manual exposure/gain once the camera starts streaming. */
    public void update() {
        if (controlsApplied || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);

        GainControl gain = portal.getCameraControl(GainControl.class);
        gain.setGain(Math.max(gain.getMinGain(), Math.min(GAIN, gain.getMaxGain())));

        controlsApplied = true;
    }
    public void setExposure(long ms, int gainValue) {
        EXPOSURE_MS = ms;
        GAIN = gainValue;
        controlsApplied = false;
    }
    public boolean isStreaming() {
        return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
    private void setValidIDs(int... tagIds) {
        validIDs.clear();
        for (int id : tagIds) {
            validIDs.add(id);
        }
    }
    private int validIDsDetected() {
        Set<Integer> tagsDetected = new LinkedHashSet<>();
        for (AprilTagDetection tag : getDetections()) {
            if (validIDs.contains(tag.id)) tagsDetected.add(tag.id);
        }
        return tagsDetected.size();
    }

    public boolean hiveTippedFor(Poses.Zone zone) {
        switch (zone) {
            case RED_SCORING:
                setValidIDs(30, 31, 32, 33);
                break;

            case RED_AUDIENCE:
                setValidIDs(34, 35, 36, 37);
                break;

            case BLUE_AUDIENCE:
                setValidIDs(38, 39, 40, 41);
                break;

            case BLUE_SCORING:
                setValidIDs(42, 43, 44, 45);
                break;
        }
        boolean tipDetected = validIDsDetected() >= REQUIRED_IDS;

        if (tipDetected) {
            zoneLastTipMs.put(zone, tipTimer.milliseconds());
        }

        if (!tipDetected && tipTimer.milliseconds() - zoneLastTipMs.get(zone) <= TIP_CONFIRM_MS) return true;
        return tipDetected;
    }
    public void enableLiveStreaming() {
        PanelsCameraStream.INSTANCE.startStream(portal, 0);
        liveStreamingEnabled = true;
    }
    public void disableLiveStreaming() {
        PanelsCameraStream.INSTANCE.stopStream();
        liveStreamingEnabled = false;
    }
    public void close() {
        disableLiveStreaming();
        portal.close();
    }
}
