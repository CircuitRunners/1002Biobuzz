package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ShooterUtil;

@Configurable
public class Shooter {
    private final Flywheel flywheel;
    private final Hood hood;
    private final Turret turret;

    public enum Mode { NO_TURRET, TURRET_ONLY, SOTM }
    private Mode mode = Mode.TURRET_ONLY;
    public Shooter(HardwareMap hardwareMap) {
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    public void update() { flywheel.update(); }
    public void setTargets(Pose currentPose, Pose targetPose, double dyInches, double robotVeloX,
                           double robotVeloY, double hoodOffset, double turretOffset) {
        double distance = currentPose.distance(targetPose);
        double flywheelVelocity = ShooterUtil.getTargetFlywheelVelocity(distance);
        double hoodAngle = ShooterUtil.getHoodTargetAngle(distance);

        switch (mode) {
            case NO_TURRET:
                turret.home();
                break;
            case TURRET_ONLY:
                turret.setTarget(currentPose, targetPose, turretOffset);
                break;
            case SOTM:
                Pose aimingTarget = ShooterUtil.SOTM_Target(currentPose, targetPose, dyInches, robotVeloX, robotVeloY);
                double aimingTargetDistance = currentPose.distance(aimingTarget);
                flywheelVelocity = ShooterUtil.getTargetFlywheelVelocity(aimingTargetDistance);
                hoodAngle = ShooterUtil.getHoodTargetAngle(aimingTargetDistance);
                turret.setTarget(currentPose, aimingTarget, turretOffset);
                break;
        }

        flywheel.setTargetVelocity(flywheelVelocity);
        hood.setAngle(hoodAngle + hoodOffset);
    }
    public void setManualTargets(double flywheelVelocity, double hoodAngle) {
        flywheel.setTargetVelocity(flywheelVelocity);
        hood.setAngle(hoodAngle);
    }
    public void setControlMethod(Flywheel.Control controlMethod) {flywheel.controlMethod = controlMethod;}
    public Flywheel.Control getControlMethod() {return flywheel.controlMethod;}
    public void setMode(Mode mode) { this.mode = mode; }
    public Mode getMode() { return mode; }
    public boolean isReady() { return flywheel.isReady(); }
    public double getFlywheelVelo() { return flywheel.getVelocity(); }
    public double getFlywheelPower() { return flywheel.getPower(); }
    public double getTargetFlywheelVelo() { return flywheel.getTargetVelocity(); }
    public double getBatteryVoltage() { return flywheel.getVoltage(); }
    public void setPIDFCoefficients(PIDFCoefficients c) { flywheel.flywheelPIDF.setCoefficients(c);}
    public double getHoodAngle() { return hood.getAngle(); }
    public void setHoodAngle(double angle) { hood.setAngle(angle); }
    public double getTurretAngle() { return turret.getAngle(); }

    public void stop() {
        flywheel.stop();
        turret.home();
        hood.home();
    }

    public static class Flywheel {
        private final DcMotorEx shooter1, shooter2;
        public enum Control {PIDF, BANG_BANG}
        private Control controlMethod = Control.BANG_BANG;
        private double targetVelocity = 0.0, currentVelocity = 0.0;

        public static double MAX_VELOCITY = 1600.0;
        public static double BANG_BANG_THRESHOLD = 80;
        public static double BANG_BANG_KF = 0.0006, BANG_BANG_KV = 0.005;
        private static double VELOCITY_TOLERANCE = 50.0;
        public static final PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.002, 0.0, 0.0, 0.0006);

        public static double NOMINAL_VOLTAGE = 12.0;
        public static double VOLTAGE_FILTER_ALPHA = 0.3;
        public static double VOLTAGE_READ_INTERVAL_MS = 250;

        public PIDFController flywheelPIDF;
        private final VoltageSensor voltageSensor;
        private final ElapsedTime voltageTimer = new ElapsedTime();
        private double filteredVoltage;

        public Flywheel(HardwareMap hardwareMap) {
            shooter1 = initMotor(hardwareMap, "shooterTop");
            shooter2 = initMotor(hardwareMap, "shooterBottom");
            shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheelPIDF = new PIDFController(pidfCoefficients);
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            filteredVoltage = readVoltage();
        }
        private static DcMotorEx initMotor(HardwareMap hardwareMap, String name) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            return motor;
        }
        private double readVoltage() {
            double v = voltageSensor.getVoltage();
            return (Double.isFinite(v) && v > 1.0) ? v : NOMINAL_VOLTAGE;
        }
        private double voltageScale() {
            if (voltageTimer.milliseconds() >= VOLTAGE_READ_INTERVAL_MS) {
                voltageTimer.reset();
                filteredVoltage += VOLTAGE_FILTER_ALPHA * (readVoltage() - filteredVoltage);
            }
            return NOMINAL_VOLTAGE / filteredVoltage;
        }
        public double getVoltage() { return filteredVoltage; }

        public void update() {
            currentVelocity = getVelocity();
            switch (controlMethod) {
                case PIDF:
                    double pidfOutput = flywheelPIDF.calculate(currentVelocity, targetVelocity) * voltageScale();
                    pidfOutput = Range.clip(pidfOutput, 0.0, 1.0);
                    setPower(pidfOutput);
                    break;

                case BANG_BANG:
                    double bbOutput;
                    double error = targetVelocity - currentVelocity;

                    if (targetVelocity <= 0.0 || -error > BANG_BANG_THRESHOLD) bbOutput = 0.0;
                    else if (error > BANG_BANG_THRESHOLD) bbOutput = 1.0;
                    else bbOutput = BANG_BANG_KV * error + BANG_BANG_KF * targetVelocity;
                    bbOutput = Range.clip(bbOutput, 0.0, 1.0);

                    setPower(bbOutput);
                    break;
            }
        }

        public double getWheel1Velo() { return shooter1.getVelocity(); }
        public double getWheel2Velo() { return shooter2.getVelocity(); }

        public double getVelocity() {
            double velocity1 = Math.abs(shooter1.getVelocity());
            double velocity2 = Math.abs(shooter2.getVelocity());
            if (!Double.isFinite(velocity1) || !Double.isFinite(velocity2)) return 0.0;
            return (velocity1 + velocity2) / 2.0;
        }
        public void setTargetVelocity(double target) {
            if (!Double.isFinite(target)) target = 0.0;
            targetVelocity = Range.clip(target, 0.0, MAX_VELOCITY);
        }
        public double getTargetVelocity() { return targetVelocity; }
        public void setPower(double power) {
            shooter1.setPower(power);
            shooter2.setPower(power);
        }
        public double getPower() {
            return (shooter1.getPower() + shooter2.getPower()) / 2;
        }
        public boolean isReady() {
            return targetVelocity > 0.0 && Math.abs(currentVelocity - targetVelocity) < VELOCITY_TOLERANCE;
        }
        public void stop() {
            targetVelocity = 0.0;
            setPower(0.0);
        }
    }

    public static class Hood {
        private final Servo hoodServo;

        public static final double MIN_POSITION = 0.05, MAX_POSITION = 1.0;
        public static final double MIN_ANGLE = 6, MAX_ANGLE = 38.0;
        private static final double HOME_ANGLE = 25.0;

        public Hood(HardwareMap hardwareMap) {
            hoodServo = hardwareMap.get(Servo.class, "hood");
            hoodServo.setDirection(Servo.Direction.REVERSE);
        }

        public void setAngle(double angle) {
            if (!Double.isFinite(angle)) angle = HOME_ANGLE;
            angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
            hoodServo.setPosition(Range.scale(angle, MIN_ANGLE, MAX_ANGLE, MIN_POSITION, MAX_POSITION));
        }

        public void setPosition(double position) {
            hoodServo.setPosition(Range.clip(position, MIN_POSITION, MAX_POSITION));
        }

        public double getAngle() {
            return Range.scale(hoodServo.getPosition(), MIN_POSITION, MAX_POSITION, MIN_ANGLE, MAX_ANGLE);
        }

        public double getPosition() { return hoodServo.getPosition(); }
        public void home() { setAngle(HOME_ANGLE); }
    }

    public static class Turret {
        private final Servo turret1, turret2;
        public static double MIN_POSITION = 0.0, MAX_POSITION = 1.0;
        public static double MIN_ANGLE = -158.0, MAX_ANGLE = 158.0;
        private static final double HOME_ANGLE = 0.0;

        public Turret(HardwareMap hardwareMap) {
            turret1 = hardwareMap.get(Servo.class, "turret1");
            turret2 = hardwareMap.get(Servo.class, "turret2");
        }

        public void setTarget(Pose currentPose, Pose targetPose, double offset) {
            double targetAngle = Math.atan2(targetPose.y() - currentPose.y(), targetPose.x() - currentPose.x());
            double robotRelativeAngle = targetAngle - currentPose.heading() + Math.toRadians(offset);
            setAngle(Math.toDegrees(AngleUnit.normalizeRadians(robotRelativeAngle)));
        }

        public void setAngle(double angle) {
            if (!Double.isFinite(angle)) angle = HOME_ANGLE;
            angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
            setPosition(Range.scale(angle, MIN_ANGLE, MAX_ANGLE, MIN_POSITION, MAX_POSITION));
        }

        public void setPosition(double position) {
            position = Range.clip(position, MIN_POSITION, MAX_POSITION);
            turret1.setPosition(position);
            turret2.setPosition(position);
        }

        public double getAngle() {
            return Range.scale(getPosition(), MIN_POSITION, MAX_POSITION, MIN_ANGLE, MAX_ANGLE);
        }

        public double getPosition() { return (turret1.getPosition() + turret2.getPosition()) / 2.0; }
        public void home() { setAngle(HOME_ANGLE); }
    }
}
