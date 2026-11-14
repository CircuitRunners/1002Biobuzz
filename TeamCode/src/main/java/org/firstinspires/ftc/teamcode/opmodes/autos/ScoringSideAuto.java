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
        // ONE_TIP,
        TWO_TIP
    }

    @Override
    protected ScoringRoutineType[] routineTypes() {return ScoringRoutineType.values();}

    @Override
    protected Pose startPose() {
        switch (routineType()) {
            // case ONE_TIP:
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
//            case ONE_TIP:
//                return Routine.from(startPose())
//                           .stay()
//                               .onStart(this::shoot)
//                               .waitUntil(this::doneShooting, 7.5)
//                           .curveTo(ParkPoses.parkFromScoring, ctrl(121, 117))
//                           .build();

            case TWO_TIP:
                return Routine.from(startPose())
                           .stay()
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting, 5)
                           .curveTo(IntakePoses.intakeGardenAlign, ctrl(120, 110))
                                .interpolatedHeading()
                           .to(IntakePoses.intakeGarden)
                               .onStart(intake::intake)
                               .speed(0.3)
                           .to(IntakePoses.intakeGardenAlign)
                               .onStart(intake::intake)
                           .to(IntakePoses.intakeGarden)
                               .onStart(intake::intake)
                               .speed(0.3)
                               .pause(1.0)
                           .curveTo(ShootPoses.shootPoseAudience1, ctrl(51.25, 138.41), ctrl(95.4,73.7), ctrl(75.6,42.6))
                               .customInterpolation(Interpolator.piecewise()
                                                                .until(0.9, Interpolator.tangent.reverse())
                                                                .until(1.0, Interpolator.linear(Math.toRadians(92), Math.toRadians(90))))
                               .onStart(intake::intake)
                               .atTVal(0.5, intake::stop)
                               .speed(1.0)
                               .onArrival(this::shoot)
                               .waitUntil(this::doneShooting, 5)
                           .to(IntakePoses.intakeAudienceFlowerAlign)
                               .interpolatedHeading()
                           .to(IntakePoses.intakeAudienceFlower)
                               .onStart(() -> {intake.intake(); intake.rampDown();})
                               .pause(0.75)
                           .to(IntakePoses.intakeAudienceFlowerAlign)
                               .onStart(intake::intake)
                           .to(IntakePoses.intakeAudienceFlower)
                               .onStart(intake::intake)
                               .pause(1)
                           .to(IntakePoses.intakeAudienceFlowerAlign)
                               .onStart(intake::intake)
                           .curveTo(ShootPoses.shootPoseAudience1, ctrl(77, 39))
                               .onStart(intake::intake)
                               .onArrival(() -> {intake.stop(); intake.rampUp(); shoot();})
                               .waitUntil(this::doneShooting, 5)
                           .curveTo(ParkPoses.parkFromOnlyPark, ctrl(82, 36))
                           .build();
            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on

}