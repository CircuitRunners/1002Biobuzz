package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.routine.CompetitionAutoBase;
import org.firstinspires.ftc.teamcode.pedro.routine.Routine;
import org.firstinspires.ftc.teamcode.util.Poses.ParkPoses;
import org.firstinspires.ftc.teamcode.util.Poses.StartPoses;

import static org.firstinspires.ftc.teamcode.util.Poses.ctrl;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AudienceSideAuto.AudienceRoutineType;
@Configurable
@Autonomous(name="Audience Side Auto", group = "COMPETITION")
public class AudienceSideAuto extends CompetitionAutoBase<AudienceRoutineType> {
    public enum AudienceRoutineType {
        ONLY_PARK,
        ONE_TIP
    }

    @Override
    protected AudienceRoutineType[] routineTypes() {return AudienceRoutineType.values();}

    @Override
    protected Pose startPose() {
        switch (routineType()) {
            case ONLY_PARK:
                return StartPoses.startPoseAudienceOnlyPark;

            case ONE_TIP:
                return StartPoses.startPoseAudienceWall;

            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }

    // @formatter:off
    @Override
    protected Routine routine() {
        switch (routineType()) {
            case ONLY_PARK:
                return Routine.from(startPose())
                               .to(ParkPoses.parkFromOnlyPark)
                               .build();

            case ONE_TIP:
                return Routine.from(startPose())
                           .stay()
                               .pause(15)
                           .stay()
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting)
                           .curveTo(ParkPoses.parkFromOnlyPark, ctrl(94, 38))
                               .interpolatedHeading()
                           .build();


            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on
}