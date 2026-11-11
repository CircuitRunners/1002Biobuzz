package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Poses;

@Configurable
@TeleOp(name = "SHOOTER TESTER", group = "TESTING")
public class ShooterTester extends OpMode {

    private static final Pose REZERO_POSE = Poses.TeleOpPoses.teleOpStartingPose;

    private static final double HOOD_ADJUSTMENT = 1.0;
    private static final double FLYWHEEL_ADJUSTMENT = 25.0;

    private final double MIN_HOOD_ANGLE = Shooter.Hood.MIN_ANGLE;
    private final double MAX_HOOD_ANGLE = Shooter.Hood.MAX_ANGLE;
    private final double MAX_FLYWHEEL_VELOCITY =
            Shooter.Flywheel.MAX_VELOCITY;

    public static double hoodAngle = 10.0;
    public static double flywheelVelocity = 0.0;

    private GlobalTelemetry tele;
    private LockingMecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx player1;

    private Pose currentPose;
    private Pose targetPose;

    private boolean shooterEnabled;
    private boolean driveLocked;

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, false);

        drive = new LockingMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        follower = Constants.create(hardwareMap);
        follower.setPose(REZERO_POSE);

        player1 = new GamepadEx(gamepad1);

        clampTargets();

        tele.addLine("Shooter Tester Ready").update();
    }

    @Override
    public void start() {
        follower.setPose(REZERO_POSE);
    }

    @Override
    public void loop() {
        player1.readButtons();

        handleButtons();

        follower.update();
        currentPose = follower.pose();
        targetPose = Poses.targetHivePose(
                Poses.currentZone(currentPose)
        );

        handleDriving();
        updateShooter();
        handleIntake();
        updateTelemetry();
    }

    private void handleButtons() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(REZERO_POSE);
            gamepad1.rumble(1.0, 1.0, 200);
        }

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooterEnabled = !shooterEnabled;
            gamepad1.rumble(0.5, 0.5, 150);
        }

        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            driveLocked = !driveLocked;

            if (driveLocked) {
                drive.lock();
            } else {
                drive.unlock();
            }
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            hoodAngle += HOOD_ADJUSTMENT;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            hoodAngle -= HOOD_ADJUSTMENT;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            flywheelVelocity += FLYWHEEL_ADJUSTMENT;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            flywheelVelocity -= FLYWHEEL_ADJUSTMENT;
        }

        clampTargets();
    }

    private void updateShooter() {
        /*
         * The hood receives its target regardless of shooterEnabled.
         * The flywheel only receives power when shooterEnabled is true.
         */
        double targetFlywheelVelocity =
                shooterEnabled ? flywheelVelocity : 0.0;

        shooter.setManualTargets(
                targetFlywheelVelocity,
                hoodAngle
        );

        shooter.update();
    }

    private void handleIntake() {
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
            intake.transfer();
        } else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            intake.intake();
        } else {
            intake.stop();
        }

        intake.update();
    }

    private void handleDriving() {
        drive.driveFieldCentric(
                player1.getLeftY(),
                player1.getLeftX(),
                player1.getRightX(),
                currentPose.heading()
        );
    }

    private void updateTelemetry() {
        double distanceToGoal = currentPose.distance(targetPose);

        tele.header("SHOOTER TESTER")
                .addData("Shooter Enabled", shooterEnabled)
                .addData("Shooter Ready", shooter.isReady())
                .addLine()
                .header("POSE")
                .addData("Current Pose", currentPose)
                .addData("Goal Pose", targetPose)
                .addData("Distance To Goal", "%.2f inches", distanceToGoal)
                .addLine()
                .header("MANUAL TARGETS")
                .addData("Hood Target", "%.2f degrees", hoodAngle)
                .addData("Hood Actual", "%.2f degrees", shooter.getHoodAngle())
                .addData("Flywheel Target", "%.2f ticks/sec",
                        shooter.getTargetFlywheelVelo())
                .addData("Flywheel Actual", "%.2f ticks/sec",
                        shooter.getFlywheelVelo())
                .addLine()
                .header("CONTROLS")
                .addLine("SQUARE: Re-zero robot")
                .addLine("RIGHT BUMPER: Flywheel on/off")
                .addLine("LEFT BUMPER: Drive lock")
                .addLine("DPAD UP/DOWN: Hood angle")
                .addLine("DPAD LEFT/RIGHT: Flywheel velocity")
                .addLine("RIGHT TRIGGER: Transfer")
                .addLine("LEFT TRIGGER: Intake")
                .showLoopTime()
                .update();
    }

    private void clampTargets() {
        hoodAngle = clamp(
                hoodAngle,
                MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE
        );

        flywheelVelocity = clamp(
                flywheelVelocity,
                0.0,
                MAX_FLYWHEEL_VELOCITY
        );
    }

    private double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(value, maximum));
    }
}