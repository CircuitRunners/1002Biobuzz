package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class Poses {
    public enum Zones {RED_AUDIENCE, RED_SCORING, BLUE_AUDIENCE, BLUE_SCORING}
    public enum Alliance {RED, BLUE}
    public static Alliance currentAlliance;
    public static void setCurrentAlliance(Alliance a) {currentAlliance = a;}

    public static final PoseFactory p = PoseFactory.degrees();
    public static Pose startingPose = p.of(72, 72, 90);
    public static void setStartingPose(Pose p) {startingPose = p;}
    public static final Pose relocPose1 = p.of(72, 72, 90);

    public static Zones currentZone(Pose pose) {
        if (currentAlliance == Alliance.RED) {
            if (pose.y() > 72) return Zones.RED_AUDIENCE;
            return Zones.RED_SCORING;
        } else {
            if (pose.y() > 72) return Zones.BLUE_SCORING;
            return Zones.BLUE_AUDIENCE;
        }
    }

    public static final Pose startPoseScoring = p.of(134, 129, 90);
    public static final Pose intakeGarden = p.of(134, 133, 90);
    public static final Pose[] intakeScoringFlower = {p.of(130, 95, 0),
            p.of(87, 133, 0),
            p.of(106, 91, 0)
    };
    public static final Pose[] intakeAudienceFlower = {p.of(94, 14, 270),
            p.of(91, 36, 0)
    };
    public static final Pose shootPoseAudience1 = p.of(110, 34, 180);
    public static final Pose shootPoseScoring1 = p.of(110, 110, 180);

    public static Pose targetHivePose(Zones zone) {
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
    public static final Pose startLine1 = p.of(72, 72, 90);
    public static final Pose endLine1 = p.of(72, 108, 90);
    public static final Pose curve1 = p.of(108, 72, 180);
    public static final Pose curve1ControlPoint = p.of(108, 108, 0);
    public static final Pose startLine2 = p.of(108, 108, -90);
    public static final Pose endLine2 = p.of(108, 72, -90);

}
