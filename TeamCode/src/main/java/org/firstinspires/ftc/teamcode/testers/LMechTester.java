package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LMechTester")
public class LMechTester extends OpMode {
    private Servo servo1;
    private Servo servo2;
    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;
    @Override
    public void init() {
       servo1 = hardwareMap.get(Servo.class, "servo1");
       servo2 = hardwareMap.get(Servo.class, "servo2");
       motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
       motorRight = hardwareMap.get(DcMotorEx.class, "motorLeft");
       telemetry.addLine("Ready");
       telemetry.update();
    }

    @Override
    public void loop() {
        double RSU = 0;
        double RSL = 0;
        double LSU = 0;
        double LSL = 0;
        if (gamepad1.dpad_up) {
            LSU +=1;
            LSL +=1;
        }
        if (gamepad1.dpad_right) {
            RSU +=1;
            RSL +=1;
        }
        if (gamepad1.dpad_down) {
            LSU -= 1;
            LSL -= 1;
        }
        if (gamepad1.dpad_left) {
            RSU -=1;
            RSL -=1;
        }
        if (gamepad1.a) {
            servo1.setPosition(LSU);
            servo2.setPosition(RSU);
            telemetry.addLine("Unlocked");
        }
        if (gamepad1.b) {
            servo1.setPosition(LSL);
            servo2.setPosition(RSL);
            telemetry.addLine("Locked");
        }
        telemetry.addData("Unlocked Position Right", RSU);
        telemetry.addData("Unlocked Position Left", LSU);
        telemetry.addData("Locked Position Right", RSL);
        telemetry.addData("Locked Position Left", LSL);
        telemetry.update();
    }
}
