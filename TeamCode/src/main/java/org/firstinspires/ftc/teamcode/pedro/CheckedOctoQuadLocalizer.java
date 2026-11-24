package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

/** Uses Pedro's OctoQuad units and pod convention, with bounded calibration and sample loss. */
public final class CheckedOctoQuadLocalizer implements Localizer {
    private static final long CALIBRATION_LIMIT = TimeUnit.SECONDS.toNanos(10);
    private static final long SAMPLE_LOSS_LIMIT = TimeUnit.MILLISECONDS.toNanos(500);
    public final OctoQuad octoQuad;
    private final OctoQuad.LocalizerDataBlock sample = new OctoQuad.LocalizerDataBlock();
    private final DistanceUnit units;
    private MotionState motion = MotionState.ofVelocity(Pose.zero(), Velocity.zero());
    private Pose pendingPose;
    private long calibrationStart;
    private long lastValidSample;
    private boolean ready;

    public CheckedOctoQuadLocalizer(HardwareMap hardwareMap, OctoQuadConfig config) {
        octoQuad = hardwareMap.get(OctoQuad.class, config.name.get());
        units = config.globalDistanceUnit.get();
        int xPort = config.xPodPort.get();
        int yPort = config.yPodPort.get();
        octoQuad.setSingleEncoderDirection(xPort, config.xPodDirection.get());
        octoQuad.setSingleEncoderDirection(yPort, config.yPodDirection.get());
        float ticksPerMm = (float) (config.ticksPerUnit.get() / config.encoderResolutionUnit.get().toMm(1));
        octoQuad.setAllLocalizerParameters(xPort, yPort, ticksPerMm, ticksPerMm,
                (float) -config.offsetUnits.get().toMm(config.yPodOffset.get()),
                (float) -config.offsetUnits.get().toMm(config.xPodOffset.get()),
                config.headingScalar.get().floatValue(), config.localizerVelocityIntervalMS.get());
        octoQuad.setI2cRecoveryMode(config.i2cRecoveryMode.get());
        reset();
    }

    @Override
    public void reset() {
        octoQuad.resetLocalizerAndCalibrateIMU();
        calibrationStart = System.nanoTime();
        ready = false;
        pendingPose = null;
        motion = MotionState.ofVelocity(Pose.zero(), Velocity.zero());
    }

    private String status = "Calibrating";
    private boolean calibrationFailed;

    public String status() { return status; }

    /** One read per call. Manual driving can continue while localization is unavailable. */
    public boolean pollReady() {
        octoQuad.readLocalizerData(sample);
        if (!sample.isDataValid()) {
            boolean imuFault = sample.crcOk && sample.localizerStatus == OctoQuad.LocalizerStatus.FAULT_NO_IMU;
            calibrationFailed = !ready && (imuFault || System.nanoTime() - calibrationStart >= CALIBRATION_LIMIT);
            if (ready && !imuFault && System.nanoTime() - lastValidSample < SAMPLE_LOSS_LIMIT) return true;
            status = calibrationFailed ? "Calibration failed: " + sample.localizerStatus
                    : ready ? "Odometry unavailable: " + sample.localizerStatus : "Calibrating";
            return false;
        }
        calibrationFailed = false;
        ready = true;
        status = "Ready";
        acceptSample();
        if (pendingPose != null) {
            setPose(pendingPose);
            pendingPose = null;
        }
        return true;
    }

    /** The tuning library uses linear OpModes and interrupts their threads on STOP. */
    public static CheckedOctoQuadLocalizer forTuning(HardwareMap map, OctoQuadConfig config) {
        CheckedOctoQuadLocalizer localizer = new CheckedOctoQuadLocalizer(map, config);
        while (!localizer.pollReady()) {
            if (localizer.calibrationFailed) throw new LocalizationUnavailable(localizer.status);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("OctoQuad calibration interrupted", e);
            }
        }
        return localizer;
    }

    @Override
    public void update() {
        if (!pollReady()) throw new LocalizationUnavailable(status);
    }

    public static final class LocalizationUnavailable extends IllegalStateException {
        LocalizationUnavailable(String status) { super(status); }
    }

    private void acceptSample() {
        lastValidSample = System.nanoTime();
        Pose pose = new Pose(units.fromMm(sample.posX_mm), units.fromMm(sample.posY_mm), sample.heading_rad);
        Velocity velocity = new Velocity(units.fromMm(sample.velX_mmS), units.fromMm(sample.velY_mmS),
                sample.velHeading_radS);
        motion = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public void setPose(Pose pose) {
        if (ready) {
            octoQuad.setLocalizerPose((int) units.toMm(pose.x()), (int) units.toMm(pose.y()), (float) pose.heading());
        } else {
            pendingPose = pose;
        }
        motion = motion.withPose(pose);
    }

    @Override
    public MotionState state() {
        return motion;
    }
}
