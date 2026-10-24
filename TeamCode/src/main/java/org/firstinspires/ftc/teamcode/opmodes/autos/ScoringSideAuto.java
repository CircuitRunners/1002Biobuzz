package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.routine.AutoBase;
import org.firstinspires.ftc.teamcode.pedro.routine.Routine;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Poses;
import org.firstinspires.ftc.teamcode.util.ShooterUtil;
@Configurable

@Autonomous(name="Scoring Side Auto", group = "COMPETITION")
public class ScoringSideAuto extends AutoBase {
    private enum RoutineType {
        ONLY_PARK,
        ONE_TIP,
        TWO_TIP
    }

    private RoutineType routineType = RoutineType.values()[0];
    private int routineIndex = 0;
    private Poses.Alliance currentAlliance = Poses.Alliance.BLUE;
    public static double hoodOffset = 2.5;
    public static double turretOffset = 0;
    public static double TRANSFER_TIMEOUT_SECONDS = 2.0;
    private Intake intake;
    private Shooter shooter;
    @Override
    protected Pose startPose() {
        switch (routineType) {
            case ONLY_PARK:
                return Poses.startPoseOnlyPark;
            case ONE_TIP:
                return Poses.startPoseScoringWall[0];

            case TWO_TIP:
            default:
                return Poses.startPoseScoringGarden;
        }
    }

    @Override
    protected void onInit() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }
    @Override
    protected void initLoop() {
        if (gamepad1.dpadUpWasPressed()) routineIndex++;
        if (gamepad1.dpadDownWasPressed()) routineIndex--;
        routineType = RoutineType.values()[Math.floorMod(routineIndex, RoutineType.values().length)];

        if (gamepad1.dpadLeftWasPressed()) currentAlliance = Poses.Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) currentAlliance = Poses.Alliance.BLUE;
        Poses.setCurrentAlliance(currentAlliance);

        tele.header("ROUTING SELECTION")
            .addLine()
            .addLine("DPAD LEFT -> RED || DPAD RIGHT -> BLUE")
            .addLine("DPAD UP/DOWN -> CHANGE ROUTINE")
            .addLine()
            .addData("Current routine", routineType)
            .addData("Alliance", Poses.currentAlliance)
            .update();
    }

    @Override
    protected void onLoop() {
        intake.update();
        shooter.update();
        updateShooting();
    }

    @Override
    protected Object selectionKey() {
        return routineType;
    }

    // @formatter:off
    @Override
    protected Routine routine() {
        switch (routineType) {
            case ONLY_PARK:
                return Routine.from(startPose())
                           .to(Poses.parkFromOnlyPark)
                               .onArrival(() -> Poses.setTeleOpStartingPose(currentPose))
                           .build();
            case ONE_TIP:
                return Routine.from(startPose())
                           .to(Poses.startPoseScoringWall[1])
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting, 7.5)
                           .curveTo(Poses.parkFromScoring[0],
                                    Poses.parkFromScoring[1])
                               .onArrival(() -> Poses.setTeleOpStartingPose(currentPose))
                           .build();

            case TWO_TIP:
                return Routine.from(startPose())
                           .to(Poses.startPoseScoringWall[1])
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting, 7.5)
                           .to(Poses.intakeGarden)
                               .onStart(intake::intake)
                           .to(Poses.shootPoseAudience1)
                               .reverseTangentHeading()
                               .onStart(intake::intake)
                               .onArrival(this::shoot)
                               .waitUntil(this::doneShooting, 10.0)
                           .curveTo(Poses.intakeAudienceFlower[0],
                                    Poses.intakeAudienceFlower[1])
                               .onStart(() -> {
                                   intake.rampDown();
                                   intake.intake();
                               })
                           .to(Poses.shootPoseAudience1)
                               .onArrival(() -> {shoot(); Poses.setTeleOpStartingPose(currentPose);})
                               .waitUntil(this::doneShooting, 7.5)
                           .build();
            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on

    private enum ShotState {IDLE, SPINNING_UP, TRANSFERING, DONE}
    private ShotState shotState = ShotState.IDLE;
    private final ElapsedTime transferTimer = new ElapsedTime();
    @Override
    protected void beginLeg() {
        shotState = ShotState.IDLE;
        intake.stop();
        shooter.stop();
        super.beginLeg();
    }
    private void shoot() {
        shotState = ShotState.SPINNING_UP;
    }
    private boolean doneShooting() {
        return shotState == ShotState.DONE;
    }
    private void updateShooting() {
        if (shotState == ShotState.IDLE || shotState == ShotState.DONE) return;

        shooter.setTargets(currentPose, targetPose, ShooterUtil.DY, 0, 0, hoodOffset, turretOffset);
        switch (shotState) {
            case SPINNING_UP:
                intake.intake();
                if (shooter.isReady()) {
                    intake.transfer();
                    shotState = ShotState.TRANSFERING;
                    transferTimer.reset();
                }
                break;
            case TRANSFERING:
                if (!shooter.isReady()) intake.intake();
                else intake.transfer();

                if (!intake.transferHasBall()) {
                    if (transferTimer.seconds() > TRANSFER_TIMEOUT_SECONDS) {
                        intake.stop();
                        shooter.stop();
                        shotState = ShotState.DONE;
                    }
                } else {
                    transferTimer.reset();
                }
        }
    }
}