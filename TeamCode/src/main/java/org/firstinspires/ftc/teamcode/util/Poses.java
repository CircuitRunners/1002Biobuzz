package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class Poses {
    public enum Alliance {RED, BLUE}
    public enum Zones {RED_AUDIENCE, RED_SCORING, BLUE_AUDIENCE, BLUE_SCORING}
    public static final PoseFactory p = PoseFactory.degrees();
    public static Pose targetHivePose(Zones zone) {
        switch (zone) {
            case BLUE_SCORING:
            case RED_AUDIENCE:
                return p.of(84, 86, 0);
                // break;

            case BLUE_AUDIENCE:
            case RED_SCORING:
                return p.of(84, 58, 0);
                // break;

            default:
                return p.of(72, 72, 0);
                // break;
        }
    }
    public static final Pose startLine1 = p.of(72, 72, 90);
    public static final Pose endLine1 = p.of(72, 108, 90);
    public static final Pose curve1 = p.of(108, 72, 180);
    public static final Pose curve1ControlPoint = p.of(108, 108, 0);
    public static final Pose startLine2 = p.of(108, 108, -90);
    public static final Pose endLine2 = p.of(108, 72, -90);

}
