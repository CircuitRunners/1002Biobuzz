package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ShooterPhysics;

public class Shooter {
    private final Flywheel flywheel;
    private final Hood hood;
    private final Turret turret;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap);
    }
    public void update() {
        flywheel.update();
    }
    public void setTargets(Pose currentPose, Pose targetPose, double dy) {
        double flywheelVelocity = ShooterPhysics.getTargetFlywheelVelocity(currentPose, targetPose, dy);
        double hoodAngle = Math.toDegrees(ShooterPhysics.getHoodTargetAngle(currentPose, targetPose, dy));

        flywheel.setTargetVelocity(flywheelVelocity);
        hood.setAngle(hoodAngle);
        turret.setTarget(currentPose, targetPose);
    }
    public boolean isReady() {
        return flywheel.isReady();
    }
    public void stop() {
        flywheel.stop();
        turret.home();
        hood.home();
    }


    public static class Flywheel {
        private final DcMotorEx shooter1;
        private final DcMotorEx shooter2;
        private double targetVelocity = 0;
        private double currentVelocity;
        private static final double VELOCITY_TOLERANCE = 50;
        private static final double MAX_VELOCITY = 3000;
        private PIDFController flywheelPIDF;
        private static final PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
        public Flywheel(HardwareMap hardwareMap) {
            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            flywheelPIDF = new PIDFController(pidfCoefficients);
        }

        public void update() {
            currentVelocity = getVelocity();
            double output = flywheelPIDF.calculate(currentVelocity, targetVelocity);
            output = Range.clip(output, 0, 1);

            shooter1.setPower(output);
            shooter2.setPower(output);
        }
        public double getVelocity() {
            double velocity1 = shooter1.getVelocity();
            double velocity2 = shooter2.getVelocity();
            if (velocity1 == 0) {
                return velocity2;
            } else if (velocity2 == 0) {
                return velocity1;
            } else {
                return (velocity1 + velocity2) / 2.0;
            }
        }
        public void setTargetVelocity(double target) {
            targetVelocity = Range.clip(target, 0, MAX_VELOCITY);
        }
        public boolean isReady() {
            return Math.abs(currentVelocity - targetVelocity) < VELOCITY_TOLERANCE;
        }
        public void stop() {
            setTargetVelocity(0.0);
            shooter1.setPower(0.0);
            shooter2.setPower(0.0);
        }

    }

    public static class Hood {
        private final Servo hoodServo;
        private final double MIN_POSITION = 0.0;
        private final double MAX_POSITION = 1.0;
        private final double MIN_ANGLE = 45.0;
        private final double MAX_ANGLE = 90.0;
        private final double HOME_ANGLE = 90.0;
        public Hood(HardwareMap hardwareMap) {
            hoodServo = hardwareMap.get(Servo.class, "hood");
        }
        public void setAngle(double angle) {
            angle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
            hoodServo.setPosition(
                    Range.scale(angle, MIN_ANGLE, MAX_ANGLE, MIN_POSITION, MAX_POSITION)
            );
        }
        public double getAngle() {
            return Range.scale(hoodServo.getPosition(), MIN_POSITION, MAX_POSITION, MIN_ANGLE, MAX_ANGLE);
        }
        public double getPosition() {
            return hoodServo.getPosition();
        }
        public void home() {setAngle(HOME_ANGLE);}

    }

    public static class Turret {
        private final Servo turret1;
        private final Servo turret2;
        private final double MIN_POSITION = 0.0;
        private final double MAX_POSITION = 1.0;
        private final double MIN_ANGLE = -180.0;
        private final double MAX_ANGLE = 180.0;
        private final double HOME_ANGLE = 0.0;
        public Turret(HardwareMap hardwareMap) {
            turret1 = hardwareMap.get(Servo.class, "turret1");
            turret2 = hardwareMap.get(Servo.class, "turret2");
        }
        public void setTarget(Pose currentPose, Pose targetPose) {
            double atan2 = Math.atan2(targetPose.y() - currentPose.y(), targetPose.x() - currentPose.x());
            double robotRelative = atan2 - currentPose.heading();
            double normalized = AngleUnit.normalizeRadians(robotRelative);
            setAngle(Math.toDegrees(normalized));
        }
        public void setAngle(double angle) {
            double targetPosition = Range.scale(angle, MIN_ANGLE, MAX_ANGLE, MIN_POSITION, MAX_POSITION);
            turret1.setPosition(targetPosition);
            turret2.setPosition(targetPosition);
        }
        public double getAngle() {
            return Range.scale(getPosition(), MIN_POSITION, MAX_POSITION, MIN_ANGLE, MAX_ANGLE);
        }
        public double getPosition() {
            return (turret1.getPosition() + turret2.getPosition()) / 2;
        }
        public void home() {setAngle(HOME_ANGLE);}
    }

}
