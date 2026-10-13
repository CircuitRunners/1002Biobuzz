package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
@TeleOp(name = "Motor Runner", group = "TESTING")
public class MotorRunner extends OpMode {

    private DcMotorEx motor;
    private GamepadEx player1;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motorName");
        player1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        player1.readButtons();
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) motor.setPower(1.0);
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) motor.setPower(-1.0);
        else motor.setPower(0.0);
    }
}
