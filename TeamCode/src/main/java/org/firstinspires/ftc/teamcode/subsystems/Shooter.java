package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ShooterUtil;

@Configurable
public class Shooter {

    private final Flywheel flywheel;
    private final Hood hood;
    private final Turret turret;

    public enum Mode {
        NO_TURRET,
        TURRET_ONLY,
        SOTM
    }

    private Mode mode = Mode.TURRET_ONLY;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    public void update() {
        flywheel.update();
    }

    public void setTargets(Pose currentPose, Pose targetPose, double dyInches, double robotVeloX, double robotVeloY, double shooterOffset, double turretOffset) {
        double distance = currentPose.distance(targetPose);
        double flywheelVelocity = ShooterUtil.getTargetFlywheelVelocity(distance);
        double hoodAngle = ShooterUtil.getHoodTargetAngle(distance);

        switch (mode) {
            case NO_TURRET:
                turret.setAngle(0.0);
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

        flywheel.setTargetVelocity(flywheelVelocity + shooterOffset);
        hood.setAngle(hoodAngle);
    }

    public void setMode(Mode mode) {
        if (mode != null) {
            this.mode = mode;
        }
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isReady() {
        return flywheel.isReady();
    }

    public double getFlywheelVelo() {
        return flywheel.getVelocity();
    }

    public double getTargetFlywheelVelo() {
        return flywheel.getTargetVelocity();
    }

    public double getHoodAngle() {
        return hood.getAngle();
    }

    public double getTurretAngle() {
        return turret.getAngle();
    }

    public void stop() {
        flywheel.stop();
        turret.home();
        hood.home();
    }

    public static class Flywheel {
        private final DcMotorEx shooter1;
        private final DcMotorEx shooter2;
        private double targetVelocity = 0.0;
        private double currentVelocity = 0.0;
        private static final double VELOCITY_TOLERANCE = 100.0;
        public static final double MAX_VELOCITY = 1600.0;
        public PIDFController flywheelPIDF;
        public static final PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1.8, 0.0, 0.4, 0.0018);
        public Flywheel(HardwareMap hardwareMap) {
            shooter1 = hardwareMap.get(DcMotorEx.class, "shooterTop");
            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooter2 = hardwareMap.get(DcMotorEx.class, "shooterBottom");
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

            flywheelPIDF = new PIDFController(pidfCoefficients);
        }

        public void update() {
            currentVelocity = getVelocity();
            double output = flywheelPIDF.calculate(currentVelocity, targetVelocity);
            if (!Double.isFinite(output)) {
                output = 0.0;
            }

            output = Range.clip(output, 0.0, 1.0);
            shooter1.setPower(output);
            shooter2.setPower(output);
        }

        public double getWheel1Velo() {
            return shooter1.getVelocity();
        }

        public double getWheel2Velo() {
            return shooter2.getVelocity();
        }

        public double getVelocity() {
            double velocity1 = Math.abs(shooter1.getVelocity());
            double velocity2 = Math.abs(shooter2.getVelocity());
            if (!Double.isFinite(velocity1) || !Double.isFinite(velocity2)) {
                return 0.0;
            }
            return (velocity1 + velocity2) / 2.0;
        }

        public void setTargetVelocity(double target) {
            if (!Double.isFinite(target)) {
                target = 0.0;
            }
            targetVelocity = Range.clip(target, 0.0, MAX_VELOCITY);
        }

        public double getTargetVelocity() {
            return targetVelocity;
        }

        public boolean isReady() {
            return Math.abs(currentVelocity - targetVelocity) < VELOCITY_TOLERANCE;
        }

        public void stop() {
            targetVelocity = 0.0;
            shooter1.setPower(0.0);
            shooter2.setPower(0.0);
        }
    }

    public static class Hood {
        private final Servo hoodServo;
        public static final double MIN_POSITION = 0.90;
        public static final double MAX_POSITION = 0.02;
        public static final double MIN_ANGLE = 13.0;
        public static final double MAX_ANGLE = 39.0;
        private static final double HOME_ANGLE = MIN_ANGLE;

        public Hood(HardwareMap hardwareMap) {
            hoodServo = hardwareMap.get(Servo.class, "hood");
        }

        public void setAngle(double angle) {
            if (!Double.isFinite(angle)) {
                angle = HOME_ANGLE;
            }
            angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
            hoodServo.setPosition(Range.scale(angle, MIN_ANGLE, MAX_ANGLE, MIN_POSITION, MAX_POSITION));
        }

        public void setPosition(double position) {
            hoodServo.setPosition(Range.clip(position, 0.0, 1.0));
        }

        public double getAngle() {
            return Range.scale(hoodServo.getPosition(), MIN_POSITION, MAX_POSITION, MIN_ANGLE, MAX_ANGLE);
        }

        public double getPosition() {
            return hoodServo.getPosition();
        }

        public void home() {
            setAngle(HOME_ANGLE);
        }
    }

    public static class Turret {

        private final Servo turret1;
        private final Servo turret2;

        public static double MIN_POSITION = 0.0;
        public static double MAX_POSITION = 1.0;

        public static double MIN_ANGLE = -158.0;
        public static double MAX_ANGLE = 158.0;

        private static final double HOME_ANGLE = 0.0;

        public Turret(HardwareMap hardwareMap) {
            turret1 = hardwareMap.get(Servo.class, "turret1");
            turret2 = hardwareMap.get(Servo.class, "turret2");
        }

        public void setTarget(Pose currentPose, Pose targetPose, double offset) {
            double targetAngle = Math.atan2(targetPose.y() - currentPose.y(), targetPose.x() - currentPose.x());
            double robotRelativeAngle = targetAngle - currentPose.heading();
            double normalizedAngle = AngleUnit.normalizeRadians(robotRelativeAngle);
            setAngle(Math.toDegrees(normalizedAngle + offset));
        }

        public void setAngle(double angle) {
            if (!Double.isFinite(angle)) {
                angle = HOME_ANGLE;
            }

            angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
            double targetPosition = Range.scale(angle, MIN_ANGLE, MAX_ANGLE, MIN_POSITION, MAX_POSITION);
            setPosition(targetPosition);
        }

        public void setPosition(double position) {
            position = Range.clip(position, MIN_POSITION, MAX_POSITION);

            turret1.setPosition(position);
            turret2.setPosition(position);
        }

        public double getAngle() {
            return Range.scale(getPosition(), MIN_POSITION, MAX_POSITION, MIN_ANGLE, MAX_ANGLE);
        }

        public double getPosition() {
            return (turret1.getPosition() + turret2.getPosition()) / 2.0;
        }

        public void home() {
            setAngle(HOME_ANGLE);
        }
    }
    public void setManualTargets(double flywheelVelocity, double hoodAngle) {
        flywheel.setTargetVelocity(flywheelVelocity);
        hood.setAngle(hoodAngle);
    }
}
