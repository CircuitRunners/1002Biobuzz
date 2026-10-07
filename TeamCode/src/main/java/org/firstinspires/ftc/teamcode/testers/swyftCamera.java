package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class swyftCamera {

    // Hardware
    private final HardwareMap hardwareMap;
    private final String cameraName;

    private WebcamName webcam;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Valid AprilTags
    private final Set<Integer> validAprilTags = new LinkedHashSet<>();

    // Detection filtering
    private int requiredSamples = 3;
    private int requiredMisses = 3;

    private int successfulSamples = 0;
    private int missedSamples = 0;

    private boolean aprilTagPresent = false;

    // Camera settings
    private boolean liveViewEnabled = false;

    public swyftCamera(HardwareMap hardwareMap, String cameraName) {
        this.hardwareMap = hardwareMap;
        this.cameraName = cameraName;
    }

    // ------------------------------------------------------------
    // CAMERA CONTROL
    // ------------------------------------------------------------

    /**
     * Starts the camera and AprilTag processor.
     */
    public void start() {

        if (visionPortal != null) {
            return;
        }

        webcam = hardwareMap.get(WebcamName.class, cameraName);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .build();

        // Higher decimation = less processing power.
        aprilTag.setDecimation(3);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)

                // Explicitly use MJPEG.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)

                .addProcessor(aprilTag)

                // Can be turned on for testing.
                .enableLiveView(liveViewEnabled)

                .build();

        resetDetectionState();
    }

    /**
     * Stops the camera and AprilTag processor.
     */
    public void stop() {

        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        aprilTag = null;
        webcam = null;

        resetDetectionState();
    }

    /**
     * Turns the camera preview stream on or off.
     *
     * Set this BEFORE calling start().
     *
     * Example:
     * camera.setLiveViewEnabled(true);
     */
    public void setLiveViewEnabled(boolean enabled) {
        liveViewEnabled = enabled;
    }

    /**
     * Returns whether live view is configured to be enabled.
     */
    public boolean isLiveViewEnabled() {
        return liveViewEnabled;
    }

    /**
     * Returns whether the camera is currently running.
     */
    public boolean isRunning() {
        return visionPortal != null;
    }

    // ------------------------------------------------------------
    // APRILTAG SETTINGS
    // ------------------------------------------------------------

    /**
     * Replaces the current valid AprilTag list.
     *
     * Example:
     * camera.setValidAprilTags(38, 39, 40, 41);
     */
    public void setValidAprilTags(int... tagIds) {

        validAprilTags.clear();

        if (tagIds != null) {
            for (int tagId : tagIds) {
                validAprilTags.add(tagId);
            }
        }

        resetDetectionState();
    }

    /**
     * Adds one AprilTag to the valid list.
     */
    public void addValidAprilTag(int tagId) {
        validAprilTags.add(tagId);
        resetDetectionState();
    }

    /**
     * Removes one AprilTag from the valid list.
     */
    public void removeValidAprilTag(int tagId) {
        validAprilTags.remove(tagId);
        resetDetectionState();
    }

    /**
     * Removes all valid AprilTags.
     */
    public void clearValidAprilTags() {
        validAprilTags.clear();
        resetDetectionState();
    }

    /**
     * Sets how many consecutive successful samples
     * are required before a tag is considered present.
     */
    public void setRequiredSamples(int samples) {

        requiredSamples = Math.max(1, samples);

        resetDetectionState();
    }

    /**
     * Sets how many consecutive missed samples
     * are required before a tag is considered absent.
     */
    public void setRequiredMisses(int misses) {

        requiredMisses = Math.max(1, misses);

        resetDetectionState();
    }

    // ------------------------------------------------------------
    // APRILTAG DETECTION
    // ------------------------------------------------------------

    /**
     * Checks whether a valid AprilTag has been detected.
     *
     * Multiple consecutive samples are used so that
     * one bad camera frame does not immediately change the result.
     */
    public boolean isAprilTagPresent() {

        if (visionPortal == null || aprilTag == null) {
            return false;
        }

        if (validAprilTags.isEmpty()) {
            return false;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();

        boolean validTagFound = false;

        for (AprilTagDetection detection : detections) {

            if (validAprilTags.contains(detection.id)) {
                validTagFound = true;
                break;
            }
        }

        if (validTagFound) {

            successfulSamples++;
            missedSamples = 0;

            if (successfulSamples >= requiredSamples) {
                aprilTagPresent = true;
            }

        } else {

            missedSamples++;
            successfulSamples = 0;

            if (missedSamples >= requiredMisses) {
                aprilTagPresent = false;
            }
        }

        return aprilTagPresent;
    }

    /**
     * Returns the ID of the first currently visible valid AprilTag.
     *
     * Returns -1 if no valid tag is currently visible.
     */
    public int getDetectedAprilTagId() {

        if (visionPortal == null || aprilTag == null) {
            return -1;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            if (validAprilTags.contains(detection.id)) {
                return detection.id;
            }
        }

        return -1;
    }

    // ------------------------------------------------------------
    // TELEMETRY
    // ------------------------------------------------------------

    /**
     * Adds concise camera information to telemetry.
     */
    public void telemetry(Telemetry telemetry) {

        telemetry.addLine("Swyft Camera");
        telemetry.addLine("------------------");

        if (!isRunning()) {

            telemetry.addLine("Status: OFF");
            telemetry.addData("Valid Tags", getValidTagsString());

            return;
        }

        boolean present = isAprilTagPresent();

        telemetry.addData(
                "Status",
                present ? "TAG DETECTED" : "NO TAG"
        );

        telemetry.addData(
                "Valid Tags",
                getValidTagsString()
        );

        telemetry.addData(
                "Samples",
                "%d/%d",
                successfulSamples,
                requiredSamples
        );

        telemetry.addData(
                "Misses",
                "%d/%d",
                missedSamples,
                requiredMisses
        );

        telemetry.addData(
                "Live View",
                liveViewEnabled ? "ON" : "OFF"
        );

        List<AprilTagDetection> detections = aprilTag.getDetections();

        int visibleValidTags = 0;

        for (AprilTagDetection detection : detections) {

            if (validAprilTags.contains(detection.id)) {

                visibleValidTags++;

                telemetry.addData(
                        "Tag " + detection.id,
                        "X=%.0f Y=%.0f",
                        detection.center.x,
                        detection.center.y
                );
            }
        }

        telemetry.addData(
                "Valid Tags Visible",
                visibleValidTags
        );
    }

    /**
     * Returns the configured valid AprilTags as a readable string.
     */
    public String getValidTagsString() {

        if (validAprilTags.isEmpty()) {
            return "NONE";
        }

        List<Integer> tags = new ArrayList<>(validAprilTags);

        StringBuilder result = new StringBuilder();

        for (int i = 0; i < tags.size(); i++) {

            if (i > 0) {
                result.append(", ");
            }

            result.append(tags.get(i));
        }

        return result.toString();
    }

    // ------------------------------------------------------------
    // INTERNAL STATE
    // ------------------------------------------------------------

    /**
     * Resets the multi-sample detection state.
     */
    public void resetDetectionState() {

        successfulSamples = 0;
        missedSamples = 0;
        aprilTagPresent = false;
    }
}