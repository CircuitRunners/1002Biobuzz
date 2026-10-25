package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
@TeleOp(name = "Motor Runner", group = "TESTING")
public class MotorRunner extends OpMode {

    private DcMotorEx intake;
    private DcMotorEx transfer;
    private GamepadEx player1;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        player1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        player1.readButtons();
        if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.setPower(1.0);
        else if (player1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) transfer.setPower(1.0);
        else {
            intake.setPower(0.0);
            transfer.setPower(0.0);
        }
    }
}
