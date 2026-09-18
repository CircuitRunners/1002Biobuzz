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

@TeleOp(name = "LMecTester", group = "TESTING")
public class LMecTester extends OpMode {
    private LockingMecanumDrive drive;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean locked = false;
    private boolean fieldCentric = true;
    private TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        // LMec drive
        drive = new LockingMecanumDrive(hardwareMap);
        drive.unlock();

        telemetry.addLine("Ready");
        telemetry.update();
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
        /*
         * Panels Telemetry
         */
        panelsTelemetry.addData(
                "FL Power",
                drive.frontLeftMotor.getPower()
        );
        panelsTelemetry.addData(
                "FR Power",
                drive.frontRightMotor.getPower()
        );
        panelsTelemetry.addData(
                "BL Power",
                drive.backLeftMotor.getPower()
        );
        panelsTelemetry.addData(
                "BR Power",
                drive.backRightMotor.getPower()
        );
        panelsTelemetry.addData("Unlocked Position", LockingMecanumDrive.unlockPos);
        panelsTelemetry.addData("Locked Position", LockingMecanumDrive.lockPos);
        panelsTelemetry.addData("Locked?", locked);
        panelsTelemetry.addData("Field Centric?", fieldCentric);
        panelsTelemetry.update();
        /*
         * Driver Station Telemetry
         */
        telemetry.addData("FL Power", drive.frontLeftMotor.getPower());
        telemetry.addData("FR Power", drive.frontRightMotor.getPower());
        telemetry.addData("BL Power", drive.backLeftMotor.getPower());
        telemetry.addData("BR Power", drive.backRightMotor.getPower());
        telemetry.addData("Unlocked Position", LockingMecanumDrive.unlockPos);
        telemetry.addData("Locked Position", LockingMecanumDrive.lockPos);
        telemetry.addData("Locked?", locked);
        telemetry.addData("LMec State", drive.getState());
        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.update();
    }
}

