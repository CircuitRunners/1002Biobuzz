package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Poses;

import java.util.List;

/** Combined flywheel tester: LEFT BUMPER toggles between PIDF and bang-bang velocity control. */
@Configurable
@TeleOp(name = "Velocity Control Method", group = "TESTING")
public class VelocityControlMethod extends OpMode {
    private GlobalTelemetry tele;
    private LockingMecanumDrive lMecDrive;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;
    private GamepadEx player1;
    private List<LynxModule> hubs;

    private Pose currentPose;
    private static final double TARGET_DY_INCHES = 49.0;
    public double HOOD_OFFSET = 0, TURRET_OFFSET = -2;
    public int shooterModeIndex = 1;
    private Pose targetPose;
    private Poses.Zone currentZone;
    private boolean rampDown = false;
    private boolean driveAutoAlign = false;
    public static double targetVelocity = 0, hoodAngle = 13;

    // PIDF tunables
    public static PIDFCoefficients pidfCoefficients = Shooter.Flywheel.pidfCoefficients;
    public static double kP = pidfCoefficients.p, kI = pidfCoefficients.i, kD = pidfCoefficients.d, kF = pidfCoefficients.f;

    // Bang-bang tunables
    public static double BANG_BANG_THRESHOLD = Shooter.Flywheel.BANG_BANG_THRESHOLD;
    public static double bbKF = Shooter.Flywheel.BANG_BANG_KF, bbKV = Shooter.Flywheel.BANG_BANG_KV;

    private enum Mode { INTAKING, SHOOTING }
    private Mode mode = Mode.INTAKING;

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing").update();

        lMecDrive = new LockingMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setMode(Shooter.Mode.TURRET_ONLY);
        shooter.setControlMethod(Shooter.Flywheel.Control.BANG_BANG);
        player1 = new GamepadEx(gamepad1);
        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.TeleOpPoses.teleOpStartingPose);

        if (Poses.getCurrentAlliance() == null) Poses.setCurrentAlliance(Poses.Alliance.BLUE);

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
        targetPose = Poses.targetHivePose(currentZone);
        handleDriving();
        handleShooterMode();
        handleControlMethod();

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

    private void handleControlMethod() {
        if (!player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) return;
        if (shooter.getControlMethod() == Shooter.Flywheel.Control.PIDF) {
            shooter.setControlMethod(Shooter.Flywheel.Control.BANG_BANG);
            gamepad1.rumble(1.0, 1.0, 200);
        } else {
            shooter.setControlMethod(Shooter.Flywheel.Control.PIDF);
            gamepad1.rumble(0.2, 0.2, 200);
        }
    }

    private void handleDriving() {
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) follower.setPose(Poses.TeleOpPoses.relocPoseCenter);
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) follower.setPose(Poses.TeleOpPoses.relocPoseGardenCorner);
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(Poses.TeleOpPoses.relocPoseParkCorner);
        lMecDrive.drive(player1.getLeftY(), player1.getLeftX(), player1.getRightX());
    }

    private void handleIntakeState() {
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.intake();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.stop();

        if (player1.wasJustPressed(GamepadKeys.Button.CROSS)) rampDown = !rampDown;
        if (rampDown) intake.rampDown();
        else intake.rampUp();
    }

    private void handleShootingState() {
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) HOOD_OFFSET += 0.5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) HOOD_OFFSET -= 0.5;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) TURRET_OFFSET -= 1;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) TURRET_OFFSET += 1;

        // Push both sets of tunables so switching methods mid-run uses current dashboard values
        shooter.setPIDFCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        Shooter.Flywheel.BANG_BANG_THRESHOLD = BANG_BANG_THRESHOLD;
        Shooter.Flywheel.BANG_BANG_KF = bbKF;
        Shooter.Flywheel.BANG_BANG_KV = bbKV;

        shooter.setManualTargets(targetVelocity, hoodAngle);

        intake.unblockTransfer();
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && shooter.isReady()) intake.transfer();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.stopWithoutBlocking();
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
        tele.header("CONTROL METHOD")
                .addData("Velocity Control", shooter.getControlMethod())
                .addLine("LEFT BUMPER -> switch PIDF / BANG_BANG")
                .addLine()
                .header("POSITIONS")
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
                .addData("Flywheel Ready", shooter.isReady())
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
