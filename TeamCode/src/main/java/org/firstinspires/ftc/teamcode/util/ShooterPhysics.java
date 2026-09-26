package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class ShooterPhysics {
    private static final double TICKS_PER_REV = 103.6;
    private static final double FLYWHEEL_RADIUS = 0.04; // in meters
    private static final double FLYWHEEL_VELOCITY_CONSTANT = 0.55;
    private static final double G = 9.81;
    public static double flywheelVeloToBallVelo(double velocity) {
        double angularVelocity = (velocity / TICKS_PER_REV) * 2 * Math.PI;
        double tangentialVelocity = angularVelocity * FLYWHEEL_RADIUS;
        double theoretical = tangentialVelocity / 2;
        return theoretical * FLYWHEEL_VELOCITY_CONSTANT;
    }
    public static double ballVeloToFlywheelVelo(double velocity) {
        double theoretical = velocity / FLYWHEEL_VELOCITY_CONSTANT;
        double tangentialVelocity = theoretical * 2;
        double angularVelocity = tangentialVelocity / FLYWHEEL_RADIUS;
        return angularVelocity * TICKS_PER_REV / (2 * Math.PI);
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
        return Math.atan(dy / dx) / 2 + Math.toRadians(45.0);
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
        double theta = getHoodTargetAngle(currentPose, targetPose, dy);
        double dx = inchesToMeters(currentPose.distance(targetPose));
        double v0 = getTargetBallVelocity(currentPose, targetPose, dy);
        return dx / (v0 * Math.cos(theta));
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

                return v0 + t * v1;
            }
        }
        throw new IllegalStateException("Lookup Table misconfigured");
    }
}
