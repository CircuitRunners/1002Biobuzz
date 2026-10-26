package org.firstinspires.ftc.teamcode.testers;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(
        name = "SHOOTER TESTER",
        group = "TESTING"
)
public class ShooterTester extends OpMode {

    /*
     * Change these values to match the robot's known starting position.
     * This is the pose assigned when the robot is re-zeroed.
     */
    private static final Pose REZERO_POSE = Poses.TeleOpPoses.teleOpStartingPose;

    /*
     * Manual shooter settings.
     */
    private static final double STARTING_HOOD_ANGLE = 25.0;
    private static final double STARTING_FLYWHEEL_VELOCITY = 800.0;

    /*
     * Adjustment amounts.
     */
    private static final double HOOD_ADJUSTMENT = 1.0;
    private static final double FLYWHEEL_ADJUSTMENT = 25.0;

    private static final double MIN_HOOD_ANGLE =
            Shooter.Hood.MIN_ANGLE;

    private static final double MAX_HOOD_ANGLE =
            Shooter.Hood.MAX_ANGLE;

    private static final double MAX_FLYWHEEL_VELOCITY =
            Shooter.Flywheel.MAX_VELOCITY;

    private GlobalTelemetry tele;

    private LockingMecanumDrive drive;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx player1;

    private Pose currentPose;
    private Pose targetPose;

    public static double hoodAngle =
            10;

    public static double flywheelVelocity =
            0;

    private boolean shooterEnabled = false;
    private boolean driveLocked = false;

    @Override
    public void init() {
        tele =
                new GlobalTelemetry(
                        telemetry,
                        false
                );

        tele.addLine("Initializing Shooter Tester")
                .update();

        drive =
                new LockingMecanumDrive(hardwareMap);

        intake =
                new Intake(hardwareMap);

        shooter =
                new Shooter(hardwareMap);

        follower =
                Constants.create(hardwareMap);

        follower.setPose(REZERO_POSE);

        player1 =
                new GamepadEx(gamepad1);

        hoodAngle =
                clamp(
                        hoodAngle,
                        MIN_HOOD_ANGLE,
                        MAX_HOOD_ANGLE
                );

        flywheelVelocity =
                clamp(
                        flywheelVelocity,
                        0.0,
                        MAX_FLYWHEEL_VELOCITY
                );

        tele.addLine("Ready")
                .update();
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

        currentPose =
                follower.pose();

        targetPose =
                Poses.targetHivePose(
                        Poses.currentZone(currentPose)
                );

        handleDriving();

        if (shooterEnabled) {
            shooter.setManualTargets(
                    flywheelVelocity,
                    hoodAngle
            );

            shooter.update();

            handleIntake();
        } else {
            shooter.stop();
            intake.stop();
        }

        updateTelemetry();
    }

    private void handleButtons() {
        /*
         * Re-zero robot localization.
         *
         * SQUARE = reset follower pose to REZERO_POSE.
         */
        if (player1.wasJustPressed(
                GamepadKeys.Button.SQUARE
        )) {
            follower.setPose(REZERO_POSE);

            gamepad1.rumble(
                    1.0,
                    1.0,
                    200
            );
        }

        /*
         * RIGHT BUMPER = enable/disable the shooter.
         */
        if (player1.wasJustPressed(
                GamepadKeys.Button.RIGHT_BUMPER
        )) {
            shooterEnabled = !shooterEnabled;

            if (!shooterEnabled) {
                shooter.stop();
                intake.stop();
            }

            gamepad1.rumble(
                    0.5,
                    0.5,
                    150
            );
        }

        /*
         * LEFT BUMPER = lock/unlock the drivetrain.
         */
        if (player1.wasJustPressed(
                GamepadKeys.Button.LEFT_BUMPER
        )) {
            driveLocked = !driveLocked;

            if (driveLocked) {
                drive.lock();
            } else {
                drive.unlock();
            }
        }

        /*
         * D-PAD UP/DOWN = hood angle adjustment.
         */
        if (player1.wasJustPressed(
                GamepadKeys.Button.DPAD_UP
        )) {
            hoodAngle += HOOD_ADJUSTMENT;
        }

        if (player1.wasJustPressed(
                GamepadKeys.Button.DPAD_DOWN
        )) {
            hoodAngle -= HOOD_ADJUSTMENT;
        }

        /*
         * D-PAD RIGHT/LEFT = flywheel velocity adjustment.
         */
        if (player1.wasJustPressed(
                GamepadKeys.Button.DPAD_RIGHT
        )) {
            flywheelVelocity += FLYWHEEL_ADJUSTMENT;
        }

        if (player1.wasJustPressed(
                GamepadKeys.Button.DPAD_LEFT
        )) {
            flywheelVelocity -= FLYWHEEL_ADJUSTMENT;
        }

        hoodAngle =
                clamp(
                        hoodAngle,
                        MIN_HOOD_ANGLE,
                        MAX_HOOD_ANGLE
                );

        flywheelVelocity =
                clamp(
                        flywheelVelocity,
                        0.0,
                        MAX_FLYWHEEL_VELOCITY
                );
    }

    private void handleDriving() {
        double forward =
                player1.getLeftY();

        double strafe =
                player1.getLeftX();

        double rotate =
                player1.getRightX();

        drive.driveFieldCentric(
               forward,
                strafe,
                rotate,
                currentPose.heading()
        );
    }

    private void handleIntake() {
        /*
         * RIGHT TRIGGER = transfer/shoot.
         * LEFT TRIGGER = outtake.
         */
        if (player1.getTrigger(
                GamepadKeys.Trigger.RIGHT_TRIGGER
        ) > 0.2) {
            if (shooter.isReady()) {
                intake.transfer();
            } else {
                intake.stop();
            }
        } else if (player1.getTrigger(
                GamepadKeys.Trigger.LEFT_TRIGGER
        ) > 0.2) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }

    private void updateTelemetry() {
        double distanceToGoal =
                currentPose.distance(targetPose);

        tele.header("SHOOTER TESTER")
                .addData(
                        "Shooter Enabled",
                        shooterEnabled
                )
                .addData(
                        "Shooter Ready",
                        shooter.isReady()
                )
                .addLine()
                .header("POSE")
                .addData(
                        "Current Pose",
                        currentPose
                )
                .addData(
                        "Goal Pose",
                        targetPose
                )
                .addData(
                        "Distance To Goal",
                        "%.2f inches",
                        distanceToGoal
                )
                .addLine()
                .header("MANUAL TARGETS")
                .addData(
                        "Hood Angle",
                        "%.2f degrees",
                        hoodAngle
                )
                .addData(
                        "Flywheel Target",
                        "%.2f ticks/sec",
                        flywheelVelocity
                )
                .addData(
                        "Flywheel Actual",
                        "%.2f ticks/sec",
                        shooter.getFlywheelVelo()
                )
                .addData(
                        "Hood Actual",
                        "%.2f degrees",
                        shooter.getHoodAngle()
                )
                .addLine()
                .header("CONTROLS")
                .addLine(
                        "SQUARE: Re-zero robot"
                )
                .addLine(
                        "RIGHT BUMPER: Shooter on/off"
                )
                .addLine(
                        "LEFT BUMPER: Drive lock"
                )
                .addLine(
                        "DPAD UP/DOWN: Hood angle"
                )
                .addLine(
                        "DPAD LEFT/RIGHT: Flywheel velocity"
                )
                .addLine(
                        "RIGHT TRIGGER: Transfer"
                )
                .addLine(
                        "LEFT TRIGGER: Outtake"
                )
                .showLoopTime()
                .update();
    }

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}
