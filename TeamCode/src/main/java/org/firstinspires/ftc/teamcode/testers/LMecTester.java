package org.firstinspires.ftc.teamcode.testers;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "LMecTester", group = "TESTING")
public class LMecTester extends OpMode {
    private LockingMecanumDrive drive;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean locked = false;
    private boolean fieldCentric = true;
    private GlobalTelemetry tele;
    @Override
    public void init() {
        Logger.start(LMecTester.this);
        tele = new GlobalTelemetry(telemetry, true);

        tele.addLine("Initializing...")
            .update();
        // LMec drive
        drive = new LockingMecanumDrive(hardwareMap);
        drive.unlock();

        tele.addLine("Ready")
            .update();
    }
    @Override
    public void loop() {
        // Get Panels Gamepad
        Gamepad g1 = PanelsGamepad.INSTANCE
                .getFirstManager()
                .asCombinedFTCGamepad(gamepad1);

        /*
         * Mecanum Drive
         *
         * Same drive method as your original Panels class.
         */

        double forward = -g1.left_stick_y;
        double strafe = g1.left_stick_x;
        double rotate = g1.right_stick_x;

        drive.drive(forward, strafe, rotate);

        /*
         * Button press detection
         */
        if (g1.left_bumper && !lastLeftBumper) {
            fieldCentric = !fieldCentric;
        }
        if (g1.right_bumper && !lastRightBumper) {
            locked = !locked;
        }
        /*
         * Servo position adjustment
         */
        if (locked) {
            if (g1.dpad_up && !lastDpadUp) {
                LockingMecanumDrive.lockPos += 0.01;
            }
            if (g1.dpad_down && !lastDpadDown) {
                LockingMecanumDrive.lockPos -= 0.01;
            }
            drive.lock();
        } else {
            if (g1.dpad_up && !lastDpadUp) {
                LockingMecanumDrive.unlockPos += 0.01;
            }
            if (g1.dpad_down && !lastDpadDown) {
                LockingMecanumDrive.unlockPos -= 0.01;
            }
            drive.unlock();
        }

        /*
         * Update previous button states
         */
        lastLeftBumper = g1.left_bumper;
        lastRightBumper = g1.right_bumper;
        lastDpadUp = g1.dpad_up;
        lastDpadDown = g1.dpad_down;

        tele.addData("FL Power", drive.frontLeftMotor.getPower())
            .addData("FR Power", drive.frontRightMotor.getPower())
            .addData("BL Power", drive.backLeftMotor.getPower())
            .addData("BR Power", drive.backRightMotor.getPower())
            .addData("Unlocked Position", LockingMecanumDrive.unlockPos)
            .addData("Locked Position", LockingMecanumDrive.lockPos)
            .addData("Locked?", locked)
            .addData("LMec State", drive.getState())
            .addData("Field Centric?", fieldCentric)
            .addLine()
            .showLoopTime()
            .update();
    }
}

