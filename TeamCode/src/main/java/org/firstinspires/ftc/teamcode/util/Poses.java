package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class Poses {
    public enum Zone {RED_AUDIENCE, RED_SCORING, BLUE_AUDIENCE, BLUE_SCORING}
    public static Zone currentZone(Pose pose) {
        if (currentAlliance == Alliance.RED) {
            if (pose.y() > 72) return Zone.RED_AUDIENCE;
            else return Zone.RED_SCORING;
        } else {
            if (pose.y() > 72) return Zone.BLUE_SCORING;
            else return Zone.BLUE_AUDIENCE;
        }
    }
    public static Pose targetHivePose(Zone zone) {
        switch (zone) {
            case BLUE_SCORING:
            case RED_AUDIENCE:
                return p.of(84, 86, 0);

            case BLUE_AUDIENCE:
            case RED_SCORING:
                return p.of(84, 58, 0);
        }
        throw new IllegalStateException("No target hive pose found");
    }

    public enum Alliance {RED, BLUE}
    private static Alliance currentAlliance;
    public static void setCurrentAlliance(Alliance a) {currentAlliance = a;}
    public static Alliance getCurrentAlliance() {return currentAlliance;}

    private static final PoseFactory p = PoseFactory.degrees();
    public static class TeleOpPoses {
        public static Pose teleOpStartingPose = p.of(72, 72, 90);

        public static void setTeleOpStartingPose(Pose p) {
            teleOpStartingPose = p;
        }

        public static Pose relocPoseCenter = p.of(72, 72, 90);
        public static Pose relocPoseGardenCorner = p.of(133, 134, 180);
        public static Pose relocPoseParkCorner = p.of(133, 10, 180);
    }

    public static class StartPoses {
        public static Pose startPoseScoringGarden = p.of(134, 129, 90);
        public static Pose startPoseScoringWall = p.of(86.7, 132.2, 270);
        public static Pose startPoseAudienceOnlyPark = p.of(134, 11, 90);
        public static Pose startPoseAudienceWall = p.of(81.6, 9.6, 90);
    }

    public static class IntakePoses {
        public static Pose intakeGarden = p.of(134, 133, 90);
        public static Pose intakeScoringFlower = p.of(130, 95, 0);
        public static Pose intakeAudienceFlower = p.of(94, 14, 270);
    }

    public static class ShootPoses {
        public static Pose shootPoseAudience1 = p.of(110, 34, 180);
        public static Pose shootPoseScoring1 = p.of(110, 110, 180);
    }

    public static class ParkPoses {
        public static Pose parkFromScoring = p.of(130, 42, 270);
        public static Pose parkFromOnlyPark = p.of(134, 28, 90);
    }

    public static Pose ctrl(double x, double y) {
        return p.of(x, y, 0);
    }

}
