package org.firstinspires.ftc.teamcode.testers;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name = "LMecTester", group = "TESTING")
public class LMecTester extends OpMode {
    private Servo lockingServo;
    private MecanumDrive drive;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean locked = false;
    private boolean fieldCentric = true;
    private double lockedPos = 0.63;
    private double unlockedPos = 0.1;
    private TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        // Servo
        lockingServo = hardwareMap.get(Servo.class, "lockingServo");
        // Mecanum drive
        drive = new MecanumDrive(hardwareMap);
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
                lockedPos += 0.01;
            }
            if (g1.dpad_down && !lastDpadDown) {
                lockedPos -= 0.01;
            }
            lockingServo.setPosition(lockedPos);
        } else {
            if (g1.dpad_up && !lastDpadUp) {
                unlockedPos += 0.01;
            }
            if (g1.dpad_down && !lastDpadDown) {
                unlockedPos -= 0.01;
            }
            lockingServo.setPosition(unlockedPos);
        }
        /*
         * Mecanum Drive
         *
         * Same drive method as your original Panels class.
         */

        double forward = -g1.left_stick_y;
        double strafe = g1.left_stick_x;
        double rotate = g1.right_stick_x;

       if (locked){
           drive.drive(forward,0,rotate);
       }
       else{
           drive.drive(forward, strafe, rotate);
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
        panelsTelemetry.addData("Unlocked Position", unlockedPos);
        panelsTelemetry.addData("Locked Position", lockedPos);
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
        telemetry.addData("Unlocked Position", unlockedPos);
        telemetry.addData("Locked Position", lockedPos);
        telemetry.addData("Locked?", locked);
        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.update();
    }
}

