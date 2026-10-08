package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwyftCamera;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTagTester", group = "TESTING")
public class SwyftCameraTester extends OpMode {
    private SwyftCamera camera;
    private GlobalTelemetry tele;
    private long exposureMs = SwyftCamera.EXPOSURE_MS;
    private int gain = SwyftCamera.GAIN;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void init() {
        Logger.start(SwyftCameraTester.this);
        tele = new GlobalTelemetry(telemetry, false);

        camera = new SwyftCamera(hardwareMap);
        camera.enableLiveStreaming();

        tele.addLine("Ready").update();
    }

    @Override
    public void init_loop() {
        camera.update();
        tele.addData("Camera streaming", camera.isStreaming()).update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastDpadUp) exposureMs++;
        if (gamepad1.dpad_down && !lastDpadDown) exposureMs = Math.max(1, exposureMs - 1);
        if (gamepad1.dpad_right && !lastDpadRight) gain += 10;
        if (gamepad1.dpad_left && !lastDpadLeft) gain = Math.max(0, gain - 10);
        if (exposureMs != SwyftCamera.EXPOSURE_MS || gain != SwyftCamera.GAIN) {
            camera.setExposure(exposureMs, gain);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        camera.update();

        tele.addData("Exposure (ms)", exposureMs)
            .addData("Gain", gain);

        for (AprilTagDetection d : camera.getDetections()) {
            if (d.metadata == null) {
                tele.addData("Unknown tag", d.id);
                continue;
            }
            tele.header("Tag " + d.id + " (" + d.metadata.name + ")");
        }
        tele.showLoopTime().update();
    }

    @Override
    public void stop() {
        camera.close();
    }
}
