package org.firstinspires.ftc.teamcode.testers;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;




//@Disabled
@Configurable
@TeleOp(name = "OneMotorFlywheelPIDFTester", group = "TEST")
public class OneMotorFlywheelPIDFTester extends OpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.007;       // proportional gain
    public static double kI = 0.000;      // integral gain
    public static double kD = 0.000;      // derivative gain
    public static double kF = 0.00042;      // feedforward ≈ 1 / maxTicksPerSec  old is 0.00042
    public static double targetVelocity = 0; // desired speed (ticks/sec)
    public static double maxPower = 1.0;          // safety clamp

    private double vNominal = 13.00;

    private VoltageSensor controlHubBattery;



    public static double cookedLoopTargetMS = 0;

    // ===== Hardware =====
    public DcMotorEx shooter1;

    private PIDFController pidf;


    private ElapsedTime loopTimer = new ElapsedTime();
    // Removed lastTicks and lastTime since they're no longer needed for manual calculation

    @Override
    public void init() {
        //  Enable bulk reads for faster loops
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        shooter1 = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controlHubBattery = hardwareMap.get(VoltageSensor.class, "Control Hub");


        telemetry.addData("Status", "Initializing Shooter...");

        telemetry.update();



        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setSetPoint(targetVelocity);
    }

    @Override

    public void init_loop() {








    }

    @Override
    public void loop() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.clearBulkCache();
        }

        //sensors.update();

        double rPM = shooter1.getVelocity();

        // --- Update gains and setpoint ---
        pidf.setPIDF(kP, kI, kD, kF);
        //pidf.setSetPoint(targetVelocity);

        // --- Compute output and send to motor ---
        // 'rPM' (the measured velocity) is passed to the custom PIDF controller.
        double output = pidf.calculate(rPM, targetVelocity);
        output = Range.clip(output, 0, maxPower);
//        shooter1.setPower(output );
                //* (vNominal / controlHubBattery.getVoltage()));

    shooter1.setPower(1);






        // --- Telemetry ---
        double loopTime = loopTimer.milliseconds();
        telemetry.addData("Target Vel (ticks/s)", targetVelocity);
        telemetry.addData("Measured Vel (ticks/s)", (shooter1.getVelocity()));
        telemetry.addData("Output Power", output);
        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Cooked Delay (ms)", cookedLoopTargetMS);
        telemetry.addData("battery voltage", controlHubBattery.getVoltage());

        //telemetry.addData("Flywheel Velo Motor",shooter1.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("Flywheel Velo Motor / rounded",Math.round(shooter2.getVelocity(AngleUnit.DEGREES)));
//        telemetry.addData("Flywheel Velo Motor RAD",shooter2.getVelocity(AngleUnit.RADIANS));
        telemetry.update();

        // --- Cooked delay to simulate TeleOp lag (~100 ms total loop) ---
        double remaining = cookedLoopTargetMS - loopTime;
        if (remaining > 0) {
            try {
                Thread.sleep((long) remaining);
            } catch (InterruptedException ignored) {}
        }

        loopTimer.reset();
    }
}
