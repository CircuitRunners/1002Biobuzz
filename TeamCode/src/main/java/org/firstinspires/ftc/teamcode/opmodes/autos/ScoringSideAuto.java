package org.firstinspires.ftc.teamcode.opmodes.autos;

import android.provider.Settings;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.routine.AutoBase;
import org.firstinspires.ftc.teamcode.pedro.routine.Routine;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Poses;
import org.firstinspires.ftc.teamcode.util.ShooterUtil;
@Configurable

@Autonomous(name="Scoring Side Auto", group = "COMPETITION")
public class ScoringSideAuto extends AutoBase {
    private enum RoutineType {
        TWO_TIP
    }

    private RoutineType routineType = RoutineType.TWO_TIP;
    private int routineIndex = 0;
    public static double shooterOffset = 0;
    public static double turretOffset = 0;
    private Intake intake;
    private Shooter shooter;
    @Override
    protected Pose startPose() {
        return Poses.startPoseScoring;
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
        if (gamepad1.dpadLeftWasPressed()) Poses.setCurrentAlliance(Poses.Alliance.RED);
        if (gamepad1.dpadRightWasPressed()) Poses.setCurrentAlliance(Poses.Alliance.BLUE);
        routineType = RoutineType.values()[Math.floorMod(routineIndex, RoutineType.values().length)];

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
    }

    @Override
    protected Object selectionKey() {
        return routineType;
    }

    // @formatter:off
    @Override
    protected Routine routine() {
        switch (routineType) {
            case TWO_TIP:
                return Routine.from(startPose())
                           .to(startPose())
                               .onStart(this::shoot)
                               .waitUntil(this::doneShooting, 5.0)
                           .to(Poses.intakeGarden)
                               .onStart(intake::intake)
                           .to(Poses.shootPoseAudience1)
                               .reverseTangentHeading()
                               .onArrival(this::shoot)
                               .waitUntil(this::doneShooting, 5.0)
                           .curveTo(Poses.intakeAudienceFlower[0],
                                    Poses.intakeAudienceFlower[1], Poses.intakeAudienceFlower[2])
                               .onStart(() -> {
                                   intake.rampDown();
                                   intake.intake();
                               })
                           .to(Poses.shootPoseAudience1)
                               .onArrival(() -> {shoot(); Poses.setTeleOpStartingPose(currentPose);})
                               .waitUntil(this::doneShooting, 5.0)
                           .build();
            default:
                throw new IllegalStateException("New routine type has not been set");
        }
    }
    // @formatter:on

    private enum ShotState {IDLE, SPINNING_UP, TRANSFERING, DONE}
    private ShotState shotState = ShotState.IDLE;
    @Override
    protected void beginLeg() {
        super.beginLeg();
        shotState = ShotState.IDLE;
        intake.stop();
        shooter.stop();
    }
    private void shoot() {
        shotState = ShotState.SPINNING_UP;
    }
    private boolean doneShooting() {
        return shotState == ShotState.DONE;
    }
    private void updateShooting() {
        if (shotState == ShotState.IDLE || shotState == ShotState.DONE) return;

        shooter.setTargets(currentPose, targetPose, ShooterUtil.DY, 0, 0, shooterOffset,turretOffset);
        switch (shotState) {
            case SPINNING_UP:
                if (shooter.isReady()) {
                    intake.transfer();
                    shotState = ShotState.TRANSFERING;
                }
                break;
            case TRANSFERING:
                if (!intake.rampHasBalls()) {
                    intake.stop();
                    shooter.stop();
                    shotState = ShotState.DONE;
                }
        }
    }
}