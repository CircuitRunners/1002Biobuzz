package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp(name = "ShooterV2Tester", group = "Testers")
public class ShooterV2Tester extends OpMode {

    public enum ControlMode { POWER, RPM }

    private static final int N = 4;

    // ---------- Panels: general ----------
    public static ControlMode controlMode = ControlMode.RPM;
    public static boolean enabled = false;

    // ---------- Panels: targets ----------
    public static double targetRPM = 3000;
    // Per-motor manual power (POWER mode), editable live in Panels
    public static double manualPower1 = 0.5;
    public static double manualPower2 = 0.5;
    public static double manualPower3 = 0.5;
    public static double manualPower4 = 0.5;

    // ---------- Panels: PIDF (applies to all shooters) ----------
    public static double kP = 0.0003;
    public static double kI = 0.0;
    public static double kD = 0.0;
    // Feedforward: power per RPM. Start ~ 1 / (free-speed RPM), e.g. 1/6000
    public static double kF = 0.00017;
    public static double MAX_INTEGRAL_POWER = 0.3;

    // ---------- Panels: motor config (applied live) ----------
    public static double TICKS_PER_REV = 28.0;
    public static double MAX_POWER = 1.0;
    public static double MIN_POWER = 0.0;
    public static boolean shooter1Reversed = false;
    public static boolean shooter2Reversed = true;
    public static boolean shooter3Reversed = false;
    public static boolean shooter4Reversed = true;
    public static boolean brakeWhenOff = false;

    // ---------- Panels: gamepad step sizes ----------
    public static double RPM_INCREMENT = 100;
    public static double POWER_INCREMENT = 0.05;

    // ---------- Hardware ----------
    private static final String[] NAMES = {
            "s1", "s2", "s3", "s4"
    };
    private final DcMotorEx[] motors = new DcMotorEx[N];
    private TelemetryManager panels;

    // ---------- State ----------
    private final double[] rpm = new double[N];
    private final double[] power = new double[N];
    private final double[] integral = new double[N];
    private final double[] prevError = new double[N];
    private final boolean[] appliedRev = new boolean[N];
    private boolean appliedBrake;
    private double lastTime;
    private boolean prevEnabled;
    private ControlMode prevMode;

    private double[] manualPowers() {
        return new double[]{manualPower1, manualPower2, manualPower3, manualPower4};
    }

    private boolean[] reversedFlags() {
        return new boolean[]{shooter1Reversed, shooter2Reversed, shooter3Reversed, shooter4Reversed};
    }

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        for (int i = 0; i < N; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        applyMotorConfig(true);
        enabled = false;
        prevEnabled = false;
        prevMode = controlMode;
        resetPID();
        lastTime = getRuntime();

        telemetry.addLine("Flywheel tester ready (4 motors)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        applyMotorConfig(false);
        readRPM();
        handleControls();

        if (controlMode != prevMode || enabled != prevEnabled) {
            resetPID();
            prevMode = controlMode;
            prevEnabled = enabled;
        }

        if (!enabled) {
            stopFlywheels();
        } else if (controlMode == ControlMode.POWER) {
            double[] mp = manualPowers();
            for (int i = 0; i < N; i++) {
                power[i] = clamp(mp[i], MIN_POWER, MAX_POWER);
                motors[i].setPower(power[i]);
            }
        } else {
            updateRPMControl(dt);
        }

        updateTelemetry();
    }

    // ---------- Controls ----------
    private void handleControls() {
        if (gamepad1.aWasPressed()) enabled = !enabled;
        if (gamepad1.yWasPressed()) resetPID();
        if (gamepad1.bWasPressed()) {
            controlMode = (controlMode == ControlMode.POWER) ? ControlMode.RPM : ControlMode.POWER;
        }

        if (gamepad1.dpadUpWasPressed())   targetRPM += RPM_INCREMENT;
        if (gamepad1.dpadDownWasPressed()) targetRPM = Math.max(0, targetRPM - RPM_INCREMENT);

        // D-pad L/R nudges all four manual powers together
        if (gamepad1.dpadRightWasPressed()) nudgeManualPowers(POWER_INCREMENT);
        if (gamepad1.dpadLeftWasPressed())  nudgeManualPowers(-POWER_INCREMENT);
    }

    private void nudgeManualPowers(double delta) {
        manualPower1 = clamp(manualPower1 + delta, MIN_POWER, MAX_POWER);
        manualPower2 = clamp(manualPower2 + delta, MIN_POWER, MAX_POWER);
        manualPower3 = clamp(manualPower3 + delta, MIN_POWER, MAX_POWER);
        manualPower4 = clamp(manualPower4 + delta, MIN_POWER, MAX_POWER);
    }

    // ---------- Hardware config (only writes on change) ----------
    private void applyMotorConfig(boolean force) {
        boolean[] rev = reversedFlags();
        for (int i = 0; i < N; i++) {
            if (force || rev[i] != appliedRev[i]) {
                motors[i].setDirection(rev[i] ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                appliedRev[i] = rev[i];
            }
        }
        if (force || brakeWhenOff != appliedBrake) {
            DcMotor.ZeroPowerBehavior zpb = brakeWhenOff
                    ? DcMotor.ZeroPowerBehavior.BRAKE
                    : DcMotor.ZeroPowerBehavior.FLOAT;
            for (DcMotorEx m : motors) m.setZeroPowerBehavior(zpb);
            appliedBrake = brakeWhenOff;
        }
    }

    private void readRPM() {
        for (int i = 0; i < N; i++) {
            rpm[i] = TICKS_PER_REV <= 0 ? 0
                    : Math.abs(motors[i].getVelocity()) / TICKS_PER_REV * 60.0;
        }
    }

    // ---------- RPM control (PID + feedforward, per motor) ----------
    private void updateRPMControl(double dt) {
        if (TICKS_PER_REV <= 0) { stopFlywheels(); return; }

        double ff = kF * targetRPM;
        for (int i = 0; i < N; i++) {
            double err = targetRPM - rpm[i];

            integral[i] += err * dt;
            if (kI != 0) {
                double lim = MAX_INTEGRAL_POWER / Math.abs(kI);
                integral[i] = clamp(integral[i], -lim, lim);
            }

            double d = (err - prevError[i]) / dt;
            prevError[i] = err;

            power[i] = clamp(ff + kP * err + kI * integral[i] + kD * d, MIN_POWER, MAX_POWER);
            motors[i].setPower(power[i]);
        }
    }

    private void resetPID() {
        for (int i = 0; i < N; i++) {
            integral[i] = 0;
            prevError[i] = 0;
        }
    }

    private void stopFlywheels() {
        for (int i = 0; i < N; i++) {
            motors[i].setPower(0);
            power[i] = 0;
        }
    }

    @Override
    public void stop() {
        enabled = false;
        stopFlywheels();
    }

    // ---------- Telemetry ----------
    private void updateTelemetry() {
        telemetry.addData("Enabled", enabled);
        telemetry.addData("Mode", controlMode);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        for (int i = 0; i < N; i++) {
            telemetry.addData("Shooter " + (i + 1), "%.0f rpm | %.2f pwr", rpm[i], power[i]);
        }
        telemetry.addLine("A: on/off  B: mode  Y: reset PID");
        telemetry.addLine("DPAD U/D: RPM  L/R: all manual power (set each in Panels)");

        // graphable in Panels
        panels.debug("Target RPM", targetRPM);
        for (int i = 0; i < N; i++) {
            panels.debug("Shooter" + (i + 1) + " RPM", rpm[i]);
            panels.debug("Shooter" + (i + 1) + " Power", power[i]);
        }
        panels.update(telemetry);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}