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
        TWO_TIP,
        DECIDE
    }

    @Override
    protected ScoringRoutineType[] routineTypes() {return ScoringRoutineType.values();}

    @Override
    protected Pose startPose() {
        switch (routineType()) {
            // case ONE_TIP:
            case TWO_TIP:
                return StartPoses.startPoseScoringWall;
            case DECIDE:
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
                           .curveTo(ShootPoses.shootPoseAudience1, ctrl(51.25, 138.41), ctrl(103,73.7), ctrl(78.5,42.6))
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
                           .to(ParkPoses.parkFromOnlyPark)
                           .build();
            case DECIDE: {
                    // Used only if the hive tipped: finish on the audience side
                    Routine shootScoring1 = Routine.fromCurrent()
                            .stay()
                                    .onArrival(this::shoot)
                                    .waitUntil(this::doneShooting, 5)
                            .curveTo(IntakePoses.intakeAudienceFlowerAlign, ctrl(85.05, 105.14), ctrl(70.79,33.11), ctrl(99.44,33.18))
                            .tangentHeading()
                            .build();

                    Routine shootAudience1 = Routine.fromCurrent()
                            .curveTo(ShootPoses.shootPoseAudience1, ctrl(59.32, 136.7), ctrl(85.71,72.04))
                                .reverseTangentHeading()
                                .speed(1.0)
                                .onArrival(this::shoot)
                                .waitUntil(this::doneShooting, 5)
                            .curveTo(IntakePoses.intakeAudienceFlowerAlign, ctrl(78.95,39.63), ctrl(99.53,38.59))
                                .tangentHeading()
                            .build();

                    Routine shootAudience2 = Routine.fromCurrent()
                            .curveTo(ShootPoses.shootPoseAudience1, ctrl(78.95,39.63), ctrl(99.53,38.59))
                                .tangentHeading()
                                .onArrival(this::shoot)
                                .waitUntil(this::doneShooting, 5)
                            .curveTo(ParkPoses.parkFromScoring, ctrl(89.15,24.44))
                                .tangentHeading()
                            .build();

                    Routine shootScoring2 = Routine.fromCurrent()
                            .curveTo(ShootPoses.shootPoseScoring1, ctrl(64.94,31.47))
                            .reverseTangentHeading()
                            .onArrival(this::shoot)
                            .waitUntil(this::doneShooting, 5)
                            .curveTo(ParkPoses.parkFromScoring, ctrl(113.77,117.29), ctrl(95.21,18.4))
                            .tangentHeading()
                            .build();

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
                            .to(ShootPoses.shootPoseScoring1)   // scoring side shoot position (default)
                                .onStart(() -> { intake.intake(); startAiming(); })
                                .atTVal(0.5, intake::stop)
                                .waitUntil(this::sawTags, 0.25)
                                .decide(this::sawTags, shootScoring1, shootAudience1)
                            // audience flower intake
                            .to(IntakePoses.intakeAudienceFlower)
                                .onStart(() -> {intake.intake(); intake.rampDown();})
                                .pause(0.75)
                            .to(IntakePoses.intakeAudienceFlowerAlign)
                                .onStart(intake::intake)
                            .to(IntakePoses.intakeAudienceFlower)
                                .onStart(() -> { intake.intake(); startAiming(); })
                                .pause(0.75)
                                .waitUntil(this::sawTags, 0.25)
                                .decide(this::sawTags, shootAudience2, shootScoring2)
                            .build();
                }
            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on

}