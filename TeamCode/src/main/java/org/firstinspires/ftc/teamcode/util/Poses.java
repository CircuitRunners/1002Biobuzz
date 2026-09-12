package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Configurable
public class Poses {
    public static final PoseFactory p = PoseFactory.degrees();
    public static final Pose startLine1 = p.of(72, 72, 90);
    public static final Pose endLine1 = p.of(72, 108, 90);
    public static final Pose curve1 = p.of(108, 72, 180);
    public static final Pose curve1ControlPoint = p.of(108, 108, 0);
    public static final Pose startLine2 = p.of(108, 108, -90);
    public static final Pose endLine2 = p.of(108, 72, -90);
}
