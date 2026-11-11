package org.firstinspires.ftc.teamcode.testers;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Poses;

@Configurable
@TeleOp(name = "BANG BANG TESTER", group = "TESTING")
public class BangBangTester extends OpMode {

    private static final Pose REZERO_POSE = Poses.TeleOpPoses.relocPoseParkCorner;

    // =========================
    // DASHBOARD CONFIGURATION
    // =========================

    public static double MAX_VELOCITY = 1600.0;

    public static double VELOCITY_TOLERANCE = 46.0;
    public static double HYSTERESIS = 55.0;

    public static double HOOD_ADJUSTMENT = 1.0;
    public static double FLYWHEEL_ADJUSTMENT = 25.0;

    public static double MIN_HOOD_ANGLE = 3.0;
    public static double MAX_HOOD_ANGLE = 28.0;

    public static double HOOD_MIN_POSITION = 0.24;
    public static double HOOD_MAX_POSITION = 1.0;

    // =========================
    // TARGETS
    // =========================

    public static double hoodAngle = 10.0;
    public static double flywheelVelocity = 0.0;

    // =========================
    // HARDWARE
    // =========================

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private Servo hoodServo;

    private Intake intake;
    private LockingMecanumDrive drive;
    private Follower follower;

    private GlobalTelemetry tele;
    private GamepadEx player1;

    // =========================
    // STATE
    // =========================

    private double currentVelocity;
    private double velocity1;
    private double velocity2;

    private boolean motorOn = false;
    private boolean shooterEnabled = false;
    private boolean driveLocked = false;

    private Pose currentPose;

    // =========================
    // INITIALIZATION
    // =========================

    @Override
    public void init() {

        tele = new GlobalTelemetry(telemetry, false);

        player1 = new GamepadEx(gamepad1);

        // Drive
        drive = new LockingMecanumDrive(hardwareMap);

        follower = Constants.create(hardwareMap);
        follower.setPose(REZERO_POSE);

        // Intake
        intake = new Intake(hardwareMap);

        // Flywheels
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooterTop");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooterBottom");

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hood
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        clampTargets();

        tele.addLine("Bang Bang Tester Ready").update();
    }

    @Override
    public void start() {
        follower.setPose(REZERO_POSE);
    }

    // =========================
    // MAIN LOOP
    // =========================

    @Override
    public void loop() {

        player1.readButtons();

        handleButtons();

        follower.update();
        currentPose = follower.pose();

        handleDriving();

        updateFlywheel();

        updateHood();

        handleIntake();

        updateTelemetry();
    }

    // =========================
    // BUTTON CONTROLS
    // =========================

    private void handleButtons() {

        // Re-zero
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            follower.setPose(REZERO_POSE);
            gamepad1.rumble(1.0, 1.0, 200);
        }

        // Shooter enable
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {

            shooterEnabled = !shooterEnabled;

            if (!shooterEnabled) {
                motorOn = false;
            }

            gamepad1.rumble(0.5, 0.5, 150);
        }

        // Drive lock
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {

            driveLocked = !driveLocked;

            if (driveLocked) {
                drive.lock();
            } else {
                drive.unlock();
            }
        }

        // Hood adjustment
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            hoodAngle += HOOD_ADJUSTMENT;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            hoodAngle -= HOOD_ADJUSTMENT;
        }

        // Flywheel adjustment
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            flywheelVelocity += FLYWHEEL_ADJUSTMENT;
        }

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            flywheelVelocity -= FLYWHEEL_ADJUSTMENT;
        }

        clampTargets();
    }

    // =========================
    // BANG-BANG FLYWHEEL
    // =========================

    private void updateFlywheel() {

        velocity1 = Math.abs(shooter1.getVelocity());
        velocity2 = Math.abs(shooter2.getVelocity());

        currentVelocity = (velocity1 + velocity2) / 2.0;

        if (!shooterEnabled || flywheelVelocity <= 0) {

            motorOn = false;

        } else {

            double error = flywheelVelocity - currentVelocity;

            // Turn ON below the lower threshold
            if (error > HYSTERESIS) {
                motorOn = true;
            }

            // Turn OFF when reaching the upper threshold
            else if (error <= VELOCITY_TOLERANCE) {
                motorOn = false;
            }
        }

        double power = motorOn ? 1.0 : 0.0;

        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    // =========================
    // HOOD
    // =========================

    private void updateHood() {

        double position = Range.scale(
                hoodAngle,
                MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE,
                HOOD_MIN_POSITION,
                HOOD_MAX_POSITION
        );

        hoodServo.setPosition(
                Range.clip(position, HOOD_MIN_POSITION, HOOD_MAX_POSITION)
        );
    }

    // =========================
    // INTAKE / TRANSFER
    // =========================

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

    // =========================
    // DRIVE
    // =========================

    private void handleDriving() {

        drive.driveFieldCentric(
                -player1.getLeftY(),
                -player1.getLeftX(),
                player1.getRightX(),
                currentPose.heading()
        );
    }

    // =========================
    // TELEMETRY
    // =========================

    private void updateTelemetry() {

        boolean ready = shooterEnabled &&
                Math.abs(currentVelocity - flywheelVelocity) < VELOCITY_TOLERANCE;

        tele.header("BANG-BANG TESTER")

                .addData("Shooter Enabled", shooterEnabled)
                .addData("Shooter Ready", ready)
                .addData("Motor State", motorOn ? "ON" : "OFF")
                .addLine()

                .header("FLYWHEEL")

                .addData("Target Velocity", "%.2f", flywheelVelocity)
                .addData("Average Velocity", "%.2f", currentVelocity)
                .addData("Shooter 1 Velocity", "%.2f", velocity1)
                .addData("Shooter 2 Velocity", "%.2f", velocity2)
                .addData("Velocity Error", "%.2f", flywheelVelocity - currentVelocity)
                .addData("Motor Power", motorOn ? 1.0 : 0.0)
                .addLine()

                .header("BANG-BANG SETTINGS")

                .addData("Lower Threshold", "%.2f", flywheelVelocity - HYSTERESIS)
                .addData("Upper Threshold", "%.2f", flywheelVelocity - VELOCITY_TOLERANCE)
                .addData("Hysteresis", "%.2f", HYSTERESIS)
                .addData("Tolerance", "%.2f", VELOCITY_TOLERANCE)
                .addLine()

                .header("HOOD")

                .addData("Target Angle", "%.2f", hoodAngle)
                .addData("Servo Position", "%.3f", hoodServo.getPosition())
                .addLine()

                .header("DRIVE")

                .addData("Pose", currentPose)
                .addData("Drive Locked", driveLocked)
                .addLine()

                .header("CONTROLS")

                .addLine("SQUARE: Re-zero robot")
                .addLine("RIGHT BUMPER: Shooter ON/OFF")
                .addLine("LEFT BUMPER: Drive lock")
                .addLine("DPAD UP/DOWN: Hood angle")
                .addLine("DPAD LEFT/RIGHT: Flywheel velocity")
                .addLine("RIGHT TRIGGER: Transfer")
                .addLine("LEFT TRIGGER: Intake")

                .showLoopTime()
                .update();
    }

    // =========================
    // TARGET CLAMPING
    // =========================

    private void clampTargets() {

        hoodAngle = Range.clip(
                hoodAngle,
                MIN_HOOD_ANGLE,
                MAX_HOOD_ANGLE
        );

        flywheelVelocity = Range.clip(
                flywheelVelocity,
                0.0,
                MAX_VELOCITY
        );
    }

    // =========================
    // STOP
    // =========================

    @Override
    public void stop() {

        shooter1.setPower(0.0);
        shooter2.setPower(0.0);

        motorOn = false;
        shooterEnabled = false;

        intake.stop();

        drive.unlock();
    }
}
