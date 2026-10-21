package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.List;

@Configurable
@TeleOp(name = "FlywheelTester", group = "TESTING")
public class FlywheelTester extends OpMode {

    private GlobalTelemetry tele;
    private Shooter.Flywheel flywheel;
    private Shooter.Hood hood;
    public static double angle = 25;
    private Intake intake;
    private List<LynxModule> hubs;
    public static double targetTicksPerSecond = 0;
    public static double kP = 0.1;
    public static double kD = 0;
    public static double kI = 0;
    public static double kF = 0;

    public static boolean INTAKEING = false;

    @Override
    public void init() {
        Logger.start(this);
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing...")
            .update();

        intake = new Intake(hardwareMap);
        hood = new Shooter.Hood(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        flywheel = new Shooter.Flywheel(hardwareMap);

        tele.addLine("Ready")
            .update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        flywheel.flywheelPIDF.setPIDF(kP, kI, kD, kF);
        flywheel.setTargetVelocity(targetTicksPerSecond);
        flywheel.update();

        hood.setAngle(angle);
        if (INTAKEING){
            intake.transfer();
        }
        else{
            intake.stop();
        }

        tele.addData("Current Velocity", flywheel.getVelocity())
            .addData("Target Velocity", targetTicksPerSecond)
            .addData("Is Ready?", flywheel.isReady())
            .addData("Max Velocity", Shooter.Flywheel.MAX_VELOCITY)
            .addData("Wheel 1 Velo", flywheel.getWheel1Velo())
            .addData("Wheel 2 Velo", flywheel.getWheel2Velo())
            .addData("kP", kP)
            .addData("kI", kI)
            .addData("kD", kD)
            .addData("kF", kF)
            .showLoopTime()
            .update();
    }
}
