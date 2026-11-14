package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

@Configurable
@TeleOp(name = "COMPETITION TELEOP", group = "COMPETITION")
public class CompetitionTeleOp extends OpMode {
    private GlobalTelemetry tele;
    private LockingMecanumDrive lMecDrive;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx player1;
    private List<LynxModule> hubs;

    private Pose currentPose;
    private static final double TARGET_DY_INCHES = 49.0;
    public static double HOOD_OFFSET = 0, TURRET_OFFSET = -2;
    public int shooterModeIndex = 1;
    private Pose targetPose;
    private Poses.Zone currentZone;
    private boolean rampDown = false;
    private boolean driveAutoAlign = false;
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
        intake.rampUp();

        shooter = new Shooter(hardwareMap);
        shooter.setMode(Shooter.Mode.TURRET_ONLY);

        player1 = new GamepadEx(gamepad1);

        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.TeleOpPoses.teleOpStartingPose);

        if (Poses.getCurrentAlliance() == null) {
            Poses.setCurrentAlliance(Poses.Alliance.BLUE);
        }

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        tele.addLine("Ready").update();
    }

    @Override
    public void init_loop() {
        player1.readButtons();
        tele.addLine("ALLIANCE SELECTION")
                .addLine("DPAD LEFT -> RED || DPAD RIGHT -> BLUE")
                .addData("Current Alliance", Poses.getCurrentAlliance())
                .update();

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) Poses.setCurrentAlliance(Poses.Alliance.RED);
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) Poses.setCurrentAlliance(Poses.Alliance.BLUE);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        player1.readButtons();
        follower.update();
        intake.update();
        shooter.update();

        currentPose = follower.pose();
        currentZone = Poses.currentZone(currentPose);
        targetPose = Poses.targetHivePose(currentZone, currentPose);
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
            driveAutoAlign = false;
            shooter.stop();
            handleIntakeState();
        } else {
            driveAutoAlign = false;
            handleShootingState();
        }

        updateTelemetry();
    }

    private void handleDriving() {
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) follower.setPose(Poses.TeleOpPoses.relocPoseFlower);
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) follower.setPose(Poses.TeleOpPoses.relocPoseGardenCorner);
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(Poses.TeleOpPoses.relocPoseParkCorner);
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

        lMecDrive.drive(forward, strafe, rotate);
    }

    private void handleIntakeState() {
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
            intake.intake();
        } else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            intake.outtake();
        } else {
            intake.stop();
        }

        if (player1.wasJustPressed(GamepadKeys.Button.CROSS)) rampDown = !rampDown;
        if (rampDown) intake.rampDown();
        else intake.rampUp();
    }

    private void handleShootingState() {
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) HOOD_OFFSET += 0.5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) HOOD_OFFSET -= 0.5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) TURRET_OFFSET -= 2.5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) TURRET_OFFSET += 2.5;

        shooter.setTargets(currentPose, targetPose, TARGET_DY_INCHES,
                        follower.velocity().vx, follower.velocity().vy,
                HOOD_OFFSET, TURRET_OFFSET);

        intake.unblockTransfer();
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && shooter.isReady()) {
            intake.transfer();
        } else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            intake.outtake();
        } else {
            intake.stopWithoutBlocking();
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
                .addData("Alliance", Poses.getCurrentAlliance())
                .addData("Current Zone", currentZone)
                .addLine()
                .header("STATES")
                .addData("Drive State", lMecDrive.getState())
                .addData("Auto Align", driveAutoAlign)
                .addData("Intake State", intake.getState())
                .addData("Shooter State", shooter.getMode())
                .addData("TeleOp State", mode)
                .addLine()
                .header("DIAGNOSTICS")
                .addData("Battery Voltage", shooter.getBatteryVoltage())
                .addData("Flywheel Velocity", shooter.getFlywheelVelo())
                .addData("Target Flywheel Velocity", shooter.getTargetFlywheelVelo())
                .addData("Hood Angle", shooter.getHoodAngle())
                .addData("Hood Offset", HOOD_OFFSET)
                .addData("Turret Angle", shooter.getTurretAngle())
                .addData("Turret Offset", TURRET_OFFSET)
                .addData("Target DY Inches", TARGET_DY_INCHES)
                .showLoopTime()
                .update();
    }

    @Override
    public void stop() {
        Poses.TeleOpPoses.setTeleOpStartingPose(currentPose);
    }
}
