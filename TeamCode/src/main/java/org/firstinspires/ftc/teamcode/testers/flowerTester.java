package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.List;



@Configurable
@TeleOp(name = "flowerTester", group = "TESTING")
public class flowerTester extends OpMode {

    private GlobalTelemetry tele;
    private Servo ramp;
    private List<LynxModule> hubs;
    private GamepadEx player1;
    private double position = 0.5;
    public static double angle = 0.0;

    @Override
    public void init() {
        Logger.start(this);
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing...")
                .update();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        ramp = hardwareMap.get(Servo.class, "flower");
        ramp.setDirection(Servo.Direction.REVERSE);

        player1 = new GamepadEx(gamepad1);

        tele.addLine("Ready")
                .update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        player1.readButtons();
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) position += 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) position -= 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) position += 0.001;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) position -= 0.001;
        position = Range.clip(position, 0, 1);

        ramp.setPosition(position);

        tele.addData("Current Position", position)

                .showLoopTime()
                .update(); //.53 down, .95 up
    }
}
