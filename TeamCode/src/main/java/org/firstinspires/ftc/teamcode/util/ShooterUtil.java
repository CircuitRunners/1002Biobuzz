package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterUtil {

    private static final double G = 9.81;

    /*
     * Distance from the shooter exit to the target, in meters.
     * This value is only used if you need it elsewhere.
     */
    private static final double HEIGHT = 0.267;

    /*
     * Ball velocity per motor encoder tick per second.
     */
    private static final double BALL_VELO_PER_TICK = 0.0037;

    public static final double DY = 1.245;

    private ShooterUtil() {
        // Utility class; do not instantiate.
    }

    public static double flywheelVeloToBallVelo(double ticksPerSecond) {
        return ticksPerSecond * BALL_VELO_PER_TICK;
    }

    public static double ballVeloToFlywheelVelo(double ballVelo) {
        if (!Double.isFinite(ballVelo)) {
            return 0.0;
        }

        return ballVelo / BALL_VELO_PER_TICK;
    }

    /**
     * Calculates the required hood angle.
     *
     * @param currentPose current robot pose
     * @param targetPose target pose
     * @param dyInches vertical target displacement in inches
     * @return hood angle in radians
     */
    public static double getHoodTargetAngle(Pose currentPose, Pose targetPose, double dyInches) {
        double dxMeters = getDistanceMeters(currentPose, targetPose);
        double dyMeters = inchesToMeters(dyInches);

        return getHoodTargetAngleMeters(dxMeters, dyMeters);
    }

    public static double getHoodTargetAngle(double distance) {
        return getLUTValue(hoodLUT, distance);
    }

    /**
     * Internal hood-angle calculation.
     * Both dx and dy must be in meters.
     */
    private static double getHoodTargetAngleMeters(double dxMeters, double dyMeters) {
        if (!Double.isFinite(dxMeters) || !Double.isFinite(dyMeters) || dxMeters <= 0.0) {
            return Math.toRadians(Shooter.Hood.MIN_ANGLE);
        }

        double desiredAngle = Math.toRadians(45.0) - Math.atan2(dyMeters, dxMeters) / 2.0;
        return Range.clip(desiredAngle, Math.toRadians(Shooter.Hood.MIN_ANGLE), Math.toRadians(Shooter.Hood.MAX_ANGLE));
    }

    /**
     * Calculates the required ball launch velocity.
     *
     * @param currentPose current robot pose
     * @param targetPose target pose
     * @param dyInches vertical target displacement in inches
     * @return ball velocity in meters per second
     */
    public static double getTargetBallVelocity(Pose currentPose, Pose targetPose, double dyInches) {
        double dxMeters = getDistanceMeters(currentPose, targetPose);
        double dyMeters = inchesToMeters(dyInches);

        if (!Double.isFinite(dxMeters) || !Double.isFinite(dyMeters) || dxMeters <= 0.0) {
            return 0.0;
        }

        double theta = getHoodTargetAngleMeters(dxMeters, dyMeters);
        double cosine = Math.cos(theta);
        double denominator = dxMeters * Math.tan(theta) - dyMeters;

        if (!Double.isFinite(cosine) || Math.abs(cosine) < 1e-6 || !Double.isFinite(denominator) || denominator <= 0.0) {
            return 0.0;
        }

        double ballVelocity = (dxMeters / cosine) * Math.sqrt(G / (2.0 * denominator));
        if (!Double.isFinite(ballVelocity) || ballVelocity < 0.0) {
            return 0.0;
        }

        return ballVelocity;
    }

    /**
     * Calculates the required flywheel velocity.
     *
     * @param currentPose current robot pose
     * @param targetPose target pose
     * @param dyInches vertical target displacement in inches
     * @return target flywheel velocity in ticks per second
     */
    public static double getTargetFlywheelVelocity(Pose currentPose, Pose targetPose, double dyInches) {
        double ballVelocity = getTargetBallVelocity(currentPose, targetPose, dyInches);
        double flywheelVelocity = ballVeloToFlywheelVelo(ballVelocity);
        if (!Double.isFinite(flywheelVelocity)) {
            return 0.0;
        }
        return flywheelVelocity;
    }

    public static double getTargetFlywheelVelocity(double distance) {
        return getLUTValue(veloLUT, distance);
    }

    /**
     * Calculates projectile time of flight.
     *
     * @param currentPose current robot pose
     * @param targetPose target pose
     * @param dyInches vertical target displacement in inches
     * @return time of flight in seconds
     */
    public static double getTimeOfFlight(Pose currentPose, Pose targetPose, double dyInches) {
        double dxMeters = getDistanceMeters(currentPose, targetPose);
        double dyMeters = inchesToMeters(dyInches);

        double theta = getHoodTargetAngleMeters(dxMeters, dyMeters);
        double ballVelocity = getTargetBallVelocity(currentPose, targetPose, dyInches);
        double horizontalVelocity = ballVelocity * Math.cos(theta);

        if (!Double.isFinite(dxMeters) || !Double.isFinite(horizontalVelocity) || dxMeters <= 0.0 || horizontalVelocity <= 0.0) {
            return 0.0;
        }

        double timeOfFlight = dxMeters / horizontalVelocity;
        if (!Double.isFinite(timeOfFlight) || timeOfFlight < 0.0) {
            return 0.0;
        }

        return timeOfFlight;
    }

    /**
     * Calculates a target pose for shooting while moving.
     *
     * robotVeloX and robotVeloY must use the same distance units per second
     * as the Pose coordinates. If Pose uses inches, these should be inches/sec.
     *
     * @param currentPose current robot pose
     * @param targetPose stationary target pose
     * @param dyInches vertical target displacement in inches
     * @param robotVeloX robot X velocity
     * @param robotVeloY robot Y velocity
     * @return compensated target pose
     */
    public static Pose SOTM_Target(Pose currentPose, Pose targetPose, double dyInches, double robotVeloX, double robotVeloY) {
        double timeOfFlight = getTimeOfFlight(currentPose, targetPose, dyInches);

        if (!Double.isFinite(timeOfFlight)) {
            timeOfFlight = 0.0;
        }

        double targetX = targetPose.x();
        double targetY = targetPose.y();

        for (int i = 0; i < 10; i++) {
            targetX = targetPose.x() - timeOfFlight * robotVeloX;
            targetY = targetPose.y() - timeOfFlight * robotVeloY;

            Pose compensatedTarget = new Pose(targetX, targetY);

            timeOfFlight = getTimeOfFlight(currentPose, compensatedTarget, dyInches);

            if (!Double.isFinite(timeOfFlight)) {
                timeOfFlight = 0.0;
            }
        }

        return new Pose(targetX, targetY);
    }

    private static double getDistanceMeters(Pose currentPose, Pose targetPose) {
        double distanceInches = currentPose.distance(targetPose);
        double distanceMeters = inchesToMeters(distanceInches);

        if (!Double.isFinite(distanceMeters)) {
            return 0.0;
        }

        return distanceMeters;
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double getLUTValue(double[][] lut, double input) {
        if (lut == null || lut.length == 0) {
            throw new IllegalArgumentException("Lookup table is empty");
        }

        if (input < lut[0][0]) {
            return lut[0][1];
        }

        if (input > lut[lut.length - 1][0]) {
            return lut[lut.length - 1][1];
        }

        for (int i = 0; i < lut.length - 1; i++) {
            double input0 = lut[i][0];
            double input1 = lut[i + 1][0];

            if (input <= input1 && input >= input0) {
                double value0 = lut[i][1];
                double value1 = lut[i + 1][1];

                if (input1 == input0) {
                    return value0;
                }

                double t = (input - input0) / (input1 - input0);

                return value0 + t * (value1 - value0);
            }
        }

        throw new IllegalStateException("Lookup Table misconfigured");
    }

    public static double[][] veloLUT = {
            {37, 822},
            {45, 829},
            {55, 866},
            {63, 933}
    };

    public static double[][] hoodLUT = {
            {37, 32},
            {45, 35},
            {55, 36},
            {63, 38}
    };
}