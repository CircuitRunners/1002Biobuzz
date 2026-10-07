package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable

@TeleOp(name = "Swyft Camera Tester", group = "Swyft")
public class swyftCameraTester extends LinearOpMode {

    private swyftCamera camera;

    public static int validTag1 = 38;
    public static int validTag2 = 39;
    public static int validTag3 = 40;
    public static int validTag4 = 41;

    public static boolean liveViewEnabled = true;

    @Override
    public void runOpMode() {

        camera = new swyftCamera(
                hardwareMap,
                "swyft camera"
        );

        // --------------------------------------------------------
        // APRILTAG SETTINGS
        // --------------------------------------------------------

        camera.setValidAprilTags(validTag1, validTag2, validTag3, validTag4);

        camera.setRequiredSamples(3);
        camera.setRequiredMisses(3);

        // --------------------------------------------------------
        // CAMERA STREAM
        // --------------------------------------------------------

        // TRUE = camera preview stream ON for testing
        // FALSE = camera preview stream OFF
        camera.setLiveViewEnabled(liveViewEnabled);

        // --------------------------------------------------------
        // START CAMERA
        // --------------------------------------------------------

        camera.start();

        telemetry.addLine("Swyft Camera Tester");
        telemetry.addLine("------------------");
        telemetry.addLine("Camera started.");
        telemetry.addData(
                "Valid Tags",
                camera.getValidTagsString()
        );
        telemetry.addData(
                "Live View",
                camera.isLiveViewEnabled() ? "ON" : "OFF"
        );
        telemetry.addLine("");
        telemetry.addLine("Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            camera.telemetry(telemetry);

            telemetry.update();

            sleep(20);
        }

        camera.stop();
    }
}