package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
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
                return StartPoses.startPoseScoringWall;

            case TWO_TIP:
                return StartPoses.startPoseScoringGarden;

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
                           .to(IntakePoses.intakeGarden)
                               .onStart(intake::intake)
                           .to(ShootPoses.shootPoseAudience1)
                               .reverseTangentHeading()
                               .onStart(intake::intake)
                               .onArrival(this::shoot)
                               .waitUntil(this::doneShooting, 10.0)
                           .curveTo(IntakePoses.intakeAudienceFlower, ctrl(91, 36))
                               .onStart(() -> {
                                   intake.rampDown();
                                   intake.intake();
                               })
                               .pause(3)
                           .to(ShootPoses.shootPoseAudience1)
                               .onStart(intake::intake)
                               .onArrival(this::shoot)
                               .waitUntil(this::doneShooting, 7.5)
                           .build();
            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on

}