package org.firstinspires.ftc.teamcode.opmodes.teleops;

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

@TeleOp(name = "COMPETITION TELEOP", group = "COMPETITION")
public class CompetitionTeleOp extends OpMode {

    private GlobalTelemetry tele;
    private LockingMecanumDrive lMecDrive;
    private boolean locked = false;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;
    private Pose currentPose;
    private Pose targetPose;
    private Poses.Zones currentZone;
    private GamepadEx player1;
    private boolean isRedAlliance;
    private enum Mode {INTAKING, SHOOTING}
    private Mode mode = Mode.INTAKING;

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing").update();

        lMecDrive = new LockingMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        player1 = new GamepadEx(gamepad1);

        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.startingPose);

        if (Poses.currentAlliance == null) Poses.setCurrentAlliance(Poses.Alliance.BLUE);

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
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && mode == Mode.INTAKING) {
            mode = Mode.SHOOTING;
            gamepad1.rumble(0.5, 1.0, 200);
        } else if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && mode == Mode.SHOOTING) {
            mode = Mode.INTAKING;
            gamepad1.rumble(1.0, 0.5, 200);
        }

        if (mode == Mode.INTAKING) {
            shooter.stop();
            handleIntakeState();
        } else {
            handleShootingState();
        }

        telemetry();
    }

    private void handleDriving() {
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(Poses.relocPose1);
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            locked = !locked;
            if (locked) {
                lMecDrive.lock();
                gamepad1.rumble(1.0, 1.0, 200);
            }
            else {
                lMecDrive.unlock();
                gamepad1.rumble(0.2, 0.2, 200);
            }
        }

        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();
        lMecDrive.driveFieldCentric(forward, strafe, rotate, currentPose.heading());
    }

    private void handleIntakeState() {
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.intake();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.stop();
    }

    private void handleShootingState() {
        shooter.setTargetsNoTurret(currentPose, targetPose, 1.245);
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && shooter.isReady()) intake.transfer();
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) intake.outtake();
        else intake.stop();
    }

    private void telemetry() {
        tele.header("POSITIONS")
            .addData("Current Pose", currentPose)
            .addData("Target Pose", targetPose)
            .addData("Alliance", Poses.currentAlliance)
            .addData("Current Zone", currentZone)
            .addLine()
            .header("STATES")
            .addData("Drive State", lMecDrive.getState())
            .addData("Intake State", intake.getState())
            .addData("TeleOp State", mode)
            .addLine()
            .header("DIAGNOSTICS")
            .addData("Flywheel Velocity", shooter.getFlywheelVelo())
            .addData("Hood Angle", shooter.getHoodAngle())
            .addData("Turret Angle", shooter.getTurretAngle())
            .showLoopTime()
            .update();
    }
}
