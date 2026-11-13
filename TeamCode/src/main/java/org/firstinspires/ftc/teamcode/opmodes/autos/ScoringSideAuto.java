package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.routine.CompetitionAutoBase;
import org.firstinspires.ftc.teamcode.pedro.routine.Routine;
import org.firstinspires.ftc.teamcode.util.Poses.IntakePoses;
import org.firstinspires.ftc.teamcode.util.Poses.ParkPoses;
import org.firstinspires.ftc.teamcode.util.Poses.ShootPoses;
import org.firstinspires.ftc.teamcode.util.Poses.StartPoses;

import static org.firstinspires.ftc.teamcode.util.Poses.ctrl;
import static org.firstinspires.ftc.teamcode.opmodes.autos.ScoringSideAuto.ScoringRoutineType;

@Configurable
@Autonomous(name="Scoring Side Auto", group = "COMPETITION")
public class ScoringSideAuto extends CompetitionAutoBase<ScoringRoutineType> {
    public enum ScoringRoutineType {
        ONE_TIP,
        TWO_TIP
    }

    @Override
    protected ScoringRoutineType[] routineTypes() {return ScoringRoutineType.values();}

    @Override
    protected Pose startPose() {
        switch (routineType()) {
            case ONE_TIP:
            case TWO_TIP:
                return StartPoses.startPoseScoringWall;


            default:
                throw new IllegalStateException("New routine type has not been set");
        }

    }

    // @formatter:off
    @Override
    protected Routine routine() {
        switch (routineType()) {
            case ONE_TIP:
                return Routine.from(startPose())
                           .stay()
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting, 7.5)
                           .curveTo(ParkPoses.parkFromScoring, ctrl(121, 117))
                           .build();

            case TWO_TIP:
                return Routine.from(startPose())
                           .stay()
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting, 7.5)
                           .curveTo(IntakePoses.intakeGardenAlign, ctrl(120, 110))
                                .interpolatedHeading()
                           .to(IntakePoses.intakeGarden)
                               .speed(0.5)
                               .onStart(intake::intake)
                           .curveTo(ShootPoses.shootPoseScoring1, ctrl(88.29, 106.04), ctrl(141.5, 2.54), ctrl(83.74, 41.75))
                               .customInterpolation(Interpolator.piecewise()
                                                                .until(0.75, Interpolator.tangent.reverse())
                                                                .until(1.0, Interpolator.linear(60, 90)))
                               .onStart(intake::intake)
                               .onArrival(this::shoot)
                               .waitUntil(this::doneShooting, 10.0)
                           .build();
            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on

}