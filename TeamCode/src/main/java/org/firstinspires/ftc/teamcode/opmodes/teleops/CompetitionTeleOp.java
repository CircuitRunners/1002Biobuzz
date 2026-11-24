package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SwyftCamera;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;
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
    private SwyftCamera camera;
    private List<LynxModule> hubs;

    private Pose currentPose;
    private static final double TARGET_DY_INCHES = 49.0;
    public static double HOOD_OFFSET = 0, TURRET_OFFSET = 0;
    public static double CAMERA_TRIM_GAIN = 0.5, CAMERA_TRIM_DEADBAND = 0.2, CAMERA_TRIM_MAX = 15.0;
    private double cameraTrim = 0.0;
    private Poses.Zone lastTrimZone = null;
    public int shooterModeIndex = 1;
    private Pose targetPose;
    private Poses.Zone currentZone;
    private boolean poseAvailable;
    private boolean triggerReleased;
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

        player1 = new GamepadEx(gamepad1);

        camera = new SwyftCamera(hardwareMap);


        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.TeleOpPoses.teleOpStartingPose);

        if (Poses.getCurrentAlliance() == null) {
            Poses.setCurrentAlliance(Poses.Alliance.BLUE);
        }

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        tele.addLine("Checking odometry").update();
    }

    @Override
    public void init_loop() {
        player1.readButtons();
        camera.update();
        Constants.localizationReady(follower);
        tele.addData("Odometry", Constants.localizationStatus(follower)).addLine("ALLIANCE SELECTION")
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
        poseAvailable = Constants.localizationReady(follower);
        camera.update();
        handleDriving();

        currentPose = follower.pose();
        currentZone = Poses.currentZone(currentPose);
        targetPose = Poses.targetHivePose(currentZone, currentPose);
        handleShooterMode();

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (mode == Mode.INTAKING) {
                mode = Mode.SHOOTING;
                triggerReleased = player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.2;
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
            if (poseAvailable) handleShootingState();
            else {
                shooter.stop();
                intake.stop();
                triggerReleased = false;
                cameraTrim = 0;
            }
        }

        intake.update();
        shooter.update();
        updateTelemetry();
    }

    private void handleDriving() {
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) relocalize(Poses.TeleOpPoses.relocPoseFlower);
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) relocalize(Poses.TeleOpPoses.relocPoseGardenCorner);
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) relocalize(Poses.TeleOpPoses.relocPoseParkCorner);
//        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//            if (lMecDrive.getState() == LockingMecanumDrive.LMecDriveState.UNLOCKED) {
//                lMecDrive.lock();
//                gamepad1.rumble(1.0, 1.0, 200);
//            } else {
//                lMecDrive.unlock();
//                gamepad1.rumble(0.2, 0.2, 200);
//            }
//        }

        double y = player1.getLeftY(); double x = player1.getLeftX(); double r = player1.getRightX();
        double forward = y * Math.abs(y);
        double strafe = x * Math.abs(x);
        double rotate = r * Math.abs(r);

        lMecDrive.drive(0.75*forward, 0.75*strafe, rotate);
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

        updateCameraTrim();
        shooter.setTargets(currentPose, targetPose, TARGET_DY_INCHES,
                        follower.velocity().vx, follower.velocity().vy,
                        HOOD_OFFSET, TURRET_OFFSET + cameraTrim);

        boolean feedPressed = player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2;
        if (!feedPressed) triggerReleased = true;
        if (triggerReleased && feedPressed) {
            intake.transfer();
        } else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }

    private void updateCameraTrim() {
        if (currentZone != lastTrimZone) {
            cameraTrim = 0.0;
            lastTrimZone = currentZone;
        }

        if (shooter.getMode() != Shooter.Mode.CAMERA_AIM) {
            cameraTrim = 0.0;
            return;
        }

        if (Double.isNaN(camera.hiveDegreeOffset(currentZone))) {
            cameraTrim = 0;
            return;
        }
        double bearing = camera.newHiveBearing(currentZone);
        if (Double.isNaN(bearing) || Math.abs(bearing) < CAMERA_TRIM_DEADBAND) return;
        cameraTrim = Range.clip(cameraTrim + CAMERA_TRIM_GAIN * bearing, -CAMERA_TRIM_MAX, CAMERA_TRIM_MAX);
    }

    private void relocalize(Pose pose) {
        follower.setPose(pose);
        cameraTrim = 0.0;
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
        tele.addData("Odometry", Constants.localizationStatus(follower)).header("POSITIONS")
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
                .addData("Flywheel Velocity", shooter.getFlywheelVelo())
                .addData("Flywheel Power", shooter.getFlywheelPower())
                .addData("Target Flywheel Velocity", shooter.getTargetFlywheelVelo())
                .addData("Hood Angle", shooter.getHoodAngle())
                .addData("Hood Offset", HOOD_OFFSET)
                .addData("Turret Angle", shooter.getTurretAngle())
                .addData("Turret Offset", TURRET_OFFSET)
                .addData("Camera Trim", cameraTrim)
                .addData("Hive Bearing", camera.hiveDegreeOffset(currentZone))
                .addData("Target DY Inches", TARGET_DY_INCHES)
                .showLoopTime()
                .update();
    }

    @Override
    public void stop() {
        if (poseAvailable) Poses.TeleOpPoses.setTeleOpStartingPose(currentPose);
        Logger.stop();
        camera.close();
    }
}
