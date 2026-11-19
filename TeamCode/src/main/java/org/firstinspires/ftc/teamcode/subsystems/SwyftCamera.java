package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

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
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class SwyftCamera {
    public static long EXPOSURE_MS = 1;
    public static int GAIN = 300;
    private final VisionPortal portal;
    private final AprilTagProcessor aprilTag;
    private boolean liveStreamingEnabled = false;
    private boolean controlsApplied = false;
    private static final int HIVE_CLUSTER_SIZE = 4;
    private static final int REQUIRED_IDS = 2;
    private final Map<Poses.Zone, Double> zoneLastTipMs = new EnumMap<>(Poses.Zone.class);
    private final ElapsedTime tipTimer = new ElapsedTime();
    private static final double TIP_CONFIRM_MS = 400;
    private long lastHiveFrameNanos = 0;

    public SwyftCamera(HardwareMap hwmap) {
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setLensIntrinsics(799.494952, 799.832060, 644.011016, 447.804766)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true);

        aprilTag = tagBuilder.build();
        // 2 = faster, shorter range; 1 = slower, longer range
        aprilTag.setDecimation(1);

        portal = new VisionPortal.Builder()
                .setCamera(hwmap.get(WebcamName.class, "swyftCamera"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                 .setCameraResolution(new Size(1280, 800))
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
        return liveStreamingEnabled;
    }
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
    private static String clusterNameFor(Poses.Zone zone) {
        switch (zone) {
            case RED_SCORING:   return "RED SCORING";
            case RED_AUDIENCE:  return "RED AUDIENCE";
            case BLUE_AUDIENCE: return "BLUE AUDIENCE";
            case BLUE_SCORING:  return "BLUE SCORING";
            default:            return null;
        }
    }
    private AprilTagClusterDetection hiveClusterFor(Poses.Zone zone) {
        String name = clusterNameFor(zone);
        for (AprilTagDetection tag : getDetections()) {
            if (tag instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) tag;
                if (cluster.metadata.name.equals(name)) return cluster;
            }
        }
        return null;
    }
    private int hiveTagsDetected(Poses.Zone zone) {
        AprilTagClusterDetection cluster = hiveClusterFor(zone);
        if (cluster == null) return 0;
        return Math.round(cluster.percentClusterFound * HIVE_CLUSTER_SIZE / 100f);
    }
    public boolean hiveTippedFor(Poses.Zone zone) {
        boolean tipDetected = hiveTagsDetected(zone) >= REQUIRED_IDS;
        if (tipDetected) zoneLastTipMs.put(zone, tipTimer.milliseconds());
        if (!tipDetected && tipTimer.milliseconds() - zoneLastTipMs.get(zone) <= TIP_CONFIRM_MS) return true;
        return tipDetected;
    }
    public double hiveDegreeOffset(Poses.Zone zone) {
        AprilTagClusterDetection cluster = hiveClusterFor(zone);
        return cluster == null ? Double.NaN : cluster.ftcPose.bearing;
    }
    public double newHiveBearing(Poses.Zone zone) {
        AprilTagClusterDetection cluster = hiveClusterFor(zone);
        if (cluster == null || cluster.frameAcquisitionNanoTime == lastHiveFrameNanos) return Double.NaN;
        lastHiveFrameNanos = cluster.frameAcquisitionNanoTime;
        return cluster.ftcPose.bearing;
    }
    public void enableLiveStreaming() {
        PanelsCameraStream.INSTANCE.startStream(portal, 0);
        liveStreamingEnabled = true;
    }
    public void disableLiveStreaming() {
        PanelsCameraStream.INSTANCE.stopStream();
        liveStreamingEnabled = false;
    }
    public double getFPS() {
        return portal.getFps();
    }
    public void close() {
        disableLiveStreaming();
        portal.close();
    }
}
