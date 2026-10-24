package org.firstinspires.ftc.teamcode.opmodes.teleops;

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
@TeleOp(name = "COMPETITION TELEOP", group = "COMPETITION")
public class CompetitionTeleOp extends OpMode {
    private GlobalTelemetry tele;
    private LockingMecanumDrive lMecDrive;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx player1;

    private boolean isRedAlliance;
    private Pose currentPose;
    private static final double TARGET_DY_INCHES = 49.0;
    public static double shooterOffset = 0;
    public int shooterModeIndex = 1;
    public static double turretOffsets;
    private Pose targetPose;
    private Poses.Zones currentZone;
    private boolean rampDown = false;

    private enum Mode {
        INTAKING,
        SHOOTING
    }
    private Mode mode = Mode.INTAKING;

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing").update();

        lMecDrive = new LockingMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setMode(Shooter.Mode.TURRET_ONLY);
        player1 = new GamepadEx(gamepad1);
        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.teleOpStartingPose);

        if (Poses.currentAlliance == null) {
            Poses.setCurrentAlliance(Poses.Alliance.BLUE);
        }

        isRedAlliance = Poses.currentAlliance == Poses.Alliance.RED;
        tele.addLine("Ready").update();
    }

    @Override
    public void init_loop() {
        player1.readButtons();
        tele.addLine("ALLIANCE SELECTION")
                .addLine("DPAD LEFT -> RED || DPAD RIGHT -> BLUE")
                .addData("Current Alliance", Poses.currentAlliance)
                .update();

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) Poses.setCurrentAlliance(Poses.Alliance.RED);
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) Poses.setCurrentAlliance(Poses.Alliance.BLUE);
        isRedAlliance = Poses.currentAlliance == Poses.Alliance.RED;
    }

    @Override
    public void loop() {
        player1.readButtons();
        follower.update();
        intake.update();
        shooter.update();

        currentPose = follower.pose();
        currentZone = Poses.currentZone(currentPose);
        targetPose = Poses.targetHivePose(currentZone);
        handleDriving();
        handleShooterMode();

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (mode == Mode.INTAKING) {
                mode = Mode.SHOOTING;
                gamepad1.rumble(0.5, 1.0, 200);

            } else {
                mode = Mode.INTAKING;
                gamepad1.rumble(1.0, 0.5, 200);
            }
        }

        if (mode == Mode.INTAKING) {
            shooter.stop();
            handleIntakeState();
        } else {
            handleShootingState();
        }

        updateTelemetry();
    }

    private void handleDriving() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(Poses.relocPose1);
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (lMecDrive.getState() == LockingMecanumDrive.LMecDriveState.UNLOCKED) {
                lMecDrive.lock();
                gamepad1.rumble(1.0, 1.0, 200);
            } else {
                lMecDrive.unlock();
                gamepad1.rumble(0.2, 0.2, 200);
            }
        }

        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();
        lMecDrive.driveFieldCentric(-forward, -strafe, rotate, currentPose.heading());
    }

    private void handleIntakeState() {
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
            intake.intake();
        } else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            intake.outtake();
        } else {
            intake.stop();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.SHARE)) shooter.setHoodAngle(34.0);

        if (player1.wasJustPressed(GamepadKeys.Button.CROSS)) rampDown = !rampDown;
        if (rampDown) intake.rampDown();
        else intake.rampUp();
    }

    private void handleShootingState() {
        shooter.setTargets(currentPose, targetPose, TARGET_DY_INCHES,
                        follower.velocity().vx, follower.velocity().vy,
                        shooterOffset, turretOffsets);

        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && shooter.isReady()) {
            intake.transfer();
        } else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }

    private void handleShooterMode() {
        Shooter.Mode previousMode = shooter.getMode();
        if (player1.wasJustPressed(GamepadKeys.Button.OPTIONS)) shooterModeIndex++;
        shooter.setMode(Shooter.Mode.values()[Math.floorMod(shooterModeIndex, Shooter.Mode.values().length)]);
        if (shooter.getMode() != previousMode) {
            gamepad1.rumbleBlips(Math.floorMod(shooterModeIndex, Shooter.Mode.values().length));
        }
    }

    private void updateTelemetry() {
        tele.header("POSITIONS")
                .addData("Current Pose", currentPose)
                .addData("Target Pose", targetPose)
                .addData("Distance to Target", currentPose.distance(targetPose))
                .addData("Alliance", Poses.currentAlliance)
                .addData("Current Zone", currentZone)
                .addLine()
                .header("STATES")
                .addData("Drive State", lMecDrive.getState())
                .addData("Intake State", intake.getState())
                .addData("Shooter State", shooter.getMode())
                .addData("TeleOp State", mode)
                .addLine()
                .header("DIAGNOSTICS")
                .addData("Flywheel Velocity", shooter.getFlywheelVelo())
                .addData("Target Flywheel Velocity", shooter.getTargetFlywheelVelo())
                .addData("Hood Angle", shooter.getHoodAngle())
                .addData("Turret Angle", shooter.getTurretAngle())
                .addData("Target DY Inches", TARGET_DY_INCHES)
                .showLoopTime()
                .update();
    }
}
