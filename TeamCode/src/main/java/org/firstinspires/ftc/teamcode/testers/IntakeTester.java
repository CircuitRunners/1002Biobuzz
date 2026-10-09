package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.List;

@TeleOp(name = "IntakeTester", group = "TESTING")
public class IntakeTester extends OpMode {
    private Intake intake;
    private GlobalTelemetry tele;
    private GamepadEx player1;
    private List<LynxModule> hubs;
    private boolean blocked = false;
    private boolean rampDown = false;

    @Override
    public void init() {
        Logger.start(this);
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing...").update();

        intake = new Intake(hardwareMap);

        player1 = new GamepadEx(gamepad1);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        tele.addLine("Ready").update();
    }

    @Override
    public void loop() {
        player1.readButtons();

        // Blocker position
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) Intake.BLOCK_POSITION += 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) Intake.BLOCK_POSITION -= 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) Intake.UNBLOCK_POSITION += 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) Intake.UNBLOCK_POSITION -= 0.01;

        // Ramp position
        if (player1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) Intake.RAMP_DOWN += 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.X)) Intake.RAMP_DOWN -= 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.CIRCLE)) Intake.RAMP_UP += 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) Intake.RAMP_UP -= 0.01;

        // Boolean control
        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) blocked = !blocked;
        if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) rampDown = !rampDown;

        // Position sets
        if (blocked) intake.blockTransfer();
        else intake.unblockTransfer();

        if (rampDown) intake.rampDown();
        else intake.rampUp();

        // Motors
        if (player1.getLeftY() > 0.2) intake.intake();
        else if (player1.getLeftY() < -0.2) intake.outtake();
        else if (player1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) intake.transfer();

        // Telemetry
        tele.addLine("Dpad UP/DOWN for BLOCK position")
            .addLine("Dpad RIGHT/LEFT for UNBLOCK position")
            .addLine("TRIANGLE/X for RAMP DOWN position")
            .addLine("CIRCLE/SQUARE for RAMP UP position")
            .addLine("RIGHT BUMPER to toggle blocker")
            .addLine("LEFT BUMPER to toggle ramp")
            .addLine("LEFT JOYSTICK to intake/outtake")
            .addLine("RIGHT TRIGGER to transfer")
            .addLine();
        
        tele.addData("Blocked?", blocked)
            .addData("Block Pos", Intake.BLOCK_POSITION)
            .addData("Unblock Pos", Intake.UNBLOCK_POSITION)
            .addLine()
            .addData("Ramp Down?", rampDown)
            .addData("Ramp Down Pos", Intake.RAMP_DOWN)
            .addData("Ramp Up Pos", Intake.RAMP_UP)
            .addLine()
            .addData("Intake State", intake.getState())
            .showLoopTime()
            .update();
    }
}
