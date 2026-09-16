package org.firstinspires.ftc.teamcode.testers;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="LMechTester", group="TESTING")
public class LMechTester extends OpMode {
    private Servo lockingServo;
    private GamepadEx gamepad;
    private Follower follower;
    boolean locked = false;
    boolean fieldCentric = true;
    double lockedPos = 0.0;
    double unlockedPos = 0.63;
    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        lockingServo = hardwareMap.get(Servo.class, "lockingServo");
        follower = Constants.create(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepad.readButtons();

        if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) fieldCentric = !fieldCentric;
        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) locked = !locked;

        if (locked) {
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) lockedPos+=0.01;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) lockedPos-=0.01;
            lockingServo.setPosition(lockedPos);
        }
        if (!locked) {
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) unlockedPos+=0.01;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) unlockedPos-=0.01;
            lockingServo.setPosition(unlockedPos);
        }

        if (fieldCentric) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading());
            follower.manual(powers);
        } else {
            follower.manual(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }


        telemetry.addData("Unlocked Position", unlockedPos);
        telemetry.addData("Locked Position", lockedPos);
        telemetry.addData("Locked?", locked);
        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.update();
    }
}
