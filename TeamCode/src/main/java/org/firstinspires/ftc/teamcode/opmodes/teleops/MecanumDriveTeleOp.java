package org.firstinspires.ftc.teamcode.opmodes.teleops;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

//@Disabled
@TeleOp(name = "Mecanum Robot Centric")
public class MecanumDriveTeleOp extends OpMode {
    private MecanumDrive drive;
    private double speedMultiply = 0.5;
    @Override
    public void init(){
        telemetry.addLine("Initializing...");
        telemetry.update();

        drive = new MecanumDrive(hardwareMap);

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 0.5;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_right)  {
            speedMultiply = 1.0;
        }
        double forward = -1*gamepad1.left_stick_y * speedMultiply;
        double strafe  =  gamepad1.left_stick_x * speedMultiply;
        double rotate  =  gamepad1.right_stick_x * speedMultiply;
        drive.drive(forward, strafe, rotate);

        telemetry.addData("fl power", drive.frontLeftMotor.getPower());
        telemetry.addData("fr Power", drive.frontRightMotor.getPower());
        telemetry.addData("rl Power", drive.backLeftMotor.getPower());
        telemetry.addData("rr Power", drive.backRightMotor.getPower());
        telemetry.addLine("sloth");
        telemetry.update();
    }
}