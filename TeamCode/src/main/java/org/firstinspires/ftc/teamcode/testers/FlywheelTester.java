package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;

@Configurable
@TeleOp(name = "FlywheelTester", group = "TESTING")
public class FlywheelTester extends OpMode {

    private GlobalTelemetry tele;
    private Shooter.Flywheel flywheel;
    public static double targetTicksPerSecond = 0;
    public static double kP = 0;
    public static double kD = 0;
    public static double kI = 0;
    public static double kF = 0;

    @Override
    public void init() {
        Logger.start(FlywheelTester.this);
        tele = new GlobalTelemetry(telemetry, true);
        tele.addLine("Initializing...")
            .update();

        flywheel = new Shooter.Flywheel(hardwareMap);

        tele.addLine("Ready")
            .update();
    }

    @Override
    public void loop() {
        flywheel.flywheelPIDF.setPIDF(kP, kD, kI, kF);
        flywheel.setTargetVelocity(targetTicksPerSecond);
        flywheel.update();

        tele.addData("Current Velocity", flywheel.getVelocity())
            .addData("Target Velocity", targetTicksPerSecond)
            .addData("kP", kP)
            .addData("kI", kI)
            .addData("kD", kD)
            .addData("kF", kF)
            .showLoopTime()
            .update();
    }
}
