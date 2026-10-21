package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterUtil {
    private static final double TICKS_PER_REV = 145.1;
    private static final double FLYWHEEL_RADIUS = 0.036; // in meters
    private static final double FLYWHEEL_VELOCITY_CONSTANT = 0.55;
    private static final double G = 9.81;
    private static final double HEIGHT = 0.267;
    private static final double BALL_VELO_PER_TICK = 0.0037; // (dy / (t sin (theta)) + gt / (2 sin (theta))) / current TPS
    public static double flywheelVeloToBallVelo(double ticksPerSecond) {
        return ticksPerSecond * BALL_VELO_PER_TICK;
    }
    public static double ballVeloToFlywheelVelo(double ballVelo) {
        return ballVelo / BALL_VELO_PER_TICK;
    }

    /**
     * One of either angle, initial velocity, or time need
     * to be pre-determined. Angle is implemented the easiest
     * by using the right triangle formed by the poses.
     * @param currentPose
     * @param targetPose
     * @param dy
     * @return
     */
    public static double getHoodTargetAngle(Pose currentPose, Pose targetPose, double dy) {
        double dx = inchesToMeters(currentPose.distance(targetPose));
        double desired = Math.toRadians(45.0) - Math.atan(dy / dx) / 2;
        return Range.clip(desired, Math.toRadians(Shooter.Hood.MIN_ANGLE), Math.toRadians(Shooter.Hood.MAX_ANGLE));
    }
    public static double getTargetBallVelocity(Pose currentPose, Pose targetPose, double dy) {
        double theta = getHoodTargetAngle(currentPose, targetPose, dy);
        double dx = inchesToMeters(currentPose.distance(targetPose));
        return (dx / Math.cos(theta)) * Math.sqrt(G / (2 * (dx * Math.tan(theta) - dy)));
    }
    public static double getTargetFlywheelVelocity(Pose currentPose, Pose targetPose, double dy) {
        return ballVeloToFlywheelVelo(getTargetBallVelocity(currentPose, targetPose, dy));
    }
    public static double getTimeOfFlight(Pose currentPose, Pose targetPose, double dy) {
        double theta = Math.toRadians(90.0) - getHoodTargetAngle(currentPose, targetPose, dy);
        double dx = inchesToMeters(currentPose.distance(targetPose));
        double v0 = getTargetBallVelocity(currentPose, targetPose, dy);
        return dx / (v0 * Math.cos(theta));
    }
    public static Pose SOTM_Target(Pose currentPose, Pose targetPose, double dy, double robotVeloX, double robotVeloY) {
        double timeOfFlight = getTimeOfFlight(currentPose, targetPose, dy);
        double targetX = targetPose.x();
        double targetY = targetPose.y();
        for (int i = 0; i < 10; i++) {
            targetX = targetPose.x() - timeOfFlight * robotVeloX;
            targetY = targetPose.y() - timeOfFlight * robotVeloY;
            timeOfFlight = getTimeOfFlight(currentPose, new Pose(targetX, targetY), dy);
        }
        return new Pose(targetX, targetY);
    }
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    public static double getLUTValue(double[][] lut, double input) {
        if (input < lut[0][0]) return lut[0][1];
        if (input > lut[lut.length - 1][0]) return lut[lut.length - 1][1];

        for (int i = 0; i < lut.length - 1; i++) {
            double i0 = lut[i][0];
            double i1 = lut[i+1][0];

            if (input <= i1 && input >= i0) {
                double v0 = lut[i][1];
                double v1 = lut[i+1][1];
                double t = (input - i0) / (i1 - i0);

                return v0 + t * (v1 - v0);
            }
        }
        throw new IllegalStateException("Lookup Table misconfigured");
    }

    public static double[][] veloLUT = {
            {10, 0},
            {15, 0},
            {20, 0},
            {25, 0},
            {30, 0},
            {35, 0},
            {40, 0},
            {45, 0},
            {50, 0},
            {55, 0},
            {60, 0},
            {65, 0}
    };

    public static double[][] hoodLUT = {
            {10, 0},
            {15, 0},
            {20, 0},
            {25, 0},
            {30, 0},
            {35, 0},
            {40, 0},
            {45, 0},
            {50, 0},
            {55, 0},
            {60, 0},
            {65, 0}
    };
}
