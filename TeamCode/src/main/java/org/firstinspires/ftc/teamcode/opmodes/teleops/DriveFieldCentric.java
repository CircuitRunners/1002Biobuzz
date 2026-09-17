package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Poses;

@TeleOp(name = "Mecanum Field Centric")
public class DriveFieldCentric extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.startLine1);
        telemetry.addLine("Done!");
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.pose();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                currentPose.heading()
        );
        follower.manual(powers);

        if (gamepad1.squareWasPressed()) {
            follower.setPose(Poses.startLine1);
        }

        telemetry.addData("X: ", currentPose.x());
        telemetry.addData("Y: ", currentPose.y());
        telemetry.addData("Heading: ", Math.toDegrees(currentPose.heading()));
        telemetry.update();
    }
}
