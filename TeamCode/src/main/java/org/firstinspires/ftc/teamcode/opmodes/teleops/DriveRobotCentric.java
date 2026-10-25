package org.firstinspires.ftc.teamcode.opmodes.teleops;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

//@Disabled
@Configurable
@Disabled
@TeleOp(name = "Mecanum Robot Centric")
public class DriveRobotCentric extends OpMode {
    private LockingMecanumDrive drive;
    private Intake intake;

    private Shooter.Flywheel shooter;

    private double value = 1000;

    private double speedMultiply = 0.5;

    private enum Mode {
        INTAKE,
        SHOOT
    }

    private Mode currentMode = Mode.INTAKE;

    // Track previous button states to detect new presses
    private boolean previousSquare = false;
    private boolean previousCircle = false;
    private boolean previousLeftTrigger = false;
    private boolean previousRightTrigger = false;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        drive = new LockingMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter.Flywheel(hardwareMap);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        shooter.update();
        intake.update();


        if (gamepad1.dpad_up) {
            speedMultiply = 1.0;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.5;
        }

        if (gamepad1.left_bumper){
            drive.unlock();
        }
        else if (gamepad1.right_bumper){
            drive.lock();
        }


        double forward = -gamepad1.left_stick_y * speedMultiply;
        double strafe = gamepad1.left_stick_x * speedMultiply;
        double rotate = gamepad1.right_stick_x * speedMultiply;

        drive.drive(forward, strafe, rotate);


        if (gamepad1.square && !previousSquare) {
            currentMode = Mode.INTAKE;
            intake.stop();
        }

        if (gamepad1.circle && !previousCircle) {
            currentMode = Mode.SHOOT;
            intake.stop();
        }

        // Trigger threshold
        boolean leftTrigger = gamepad1.left_trigger > 0.25;
        boolean rightTrigger = gamepad1.right_trigger > 0.25;

        if (currentMode == Mode.INTAKE) {

            shooter.stop();
            if (rightTrigger) {
                intake.intake();
            }
            else if (leftTrigger) {
                intake.outtake();
            }
            else intake.stop();




        } else if (currentMode == Mode.SHOOT) {
            shooter.setTargetVelocity(value);

            if (gamepad1.triangle) {
                intake.transfer();
            } else {
                intake.stop();
            }
        }


        previousSquare = gamepad1.square;
        previousCircle = gamepad1.circle;
        previousLeftTrigger = leftTrigger;
        previousRightTrigger = rightTrigger;


        telemetry.addData("Mode", currentMode);
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Speed Multiplier", speedMultiply);
        telemetry.addData("Lmec",drive.getState());
        telemetry.addData("current velo", shooter.getVelocity());
        telemetry.addData("bottom",shooter.getWheel2Velo());
        telemetry.addData("top",shooter.getWheel1Velo());

        telemetry.addData("FL Power", drive.frontLeftMotor.getPower());
        telemetry.addData("FR Power", drive.frontRightMotor.getPower());
        telemetry.addData("RL Power", drive.backLeftMotor.getPower());
        telemetry.addData("RR Power", drive.backRightMotor.getPower());

        telemetry.update();
    }
}