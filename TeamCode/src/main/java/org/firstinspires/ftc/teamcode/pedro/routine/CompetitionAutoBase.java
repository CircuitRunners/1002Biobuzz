package org.firstinspires.ftc.teamcode.pedro.routine;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Poses;
import org.firstinspires.ftc.teamcode.util.ShooterUtil;

@Configurable
public abstract class CompetitionAutoBase<R extends Enum<R>> extends AutoBase {
    public static double HOOD_OFFSET = 1.5;
    public static double TURRET_OFFSET = -2;
    public static double TRANSFER_TIMEOUT_SECONDS = 1.0;

    protected Intake intake;
    protected Shooter shooter;

    private int routineIndex = 0;
    private Poses.Alliance currentAlliance = Poses.Alliance.BLUE;

    protected R[] routineTypes() {
        return null;
    }

    protected R routineType() {
        R[] types = routineTypes();
        if (types == null || types.length == 0) return null;
        return types[Math.floorMod(routineIndex, types.length)];
    }

    @Override
    protected Object selectionKey() {
        return routineType();
    }

    @Override
    protected void initLoop() {
        boolean up = gamepad1.dpadUpWasPressed();
        boolean down = gamepad1.dpadDownWasPressed();
        boolean left = gamepad1.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed();

        boolean horizontalHeld = gamepad1.dpad_left || gamepad1.dpad_right;
        boolean verticalHeld = gamepad1.dpad_up || gamepad1.dpad_down;

        if (!horizontalHeld) {
            if (up) routineIndex++;
            if (down) routineIndex--;
        }
        if (!verticalHeld) {
            if (left) currentAlliance = Poses.Alliance.RED;
            if (right) currentAlliance = Poses.Alliance.BLUE;
        }
        Poses.setCurrentAlliance(currentAlliance);

        tele.header("ROUTINE SELECTION")
            .addLine()
            .addLine("DPAD LEFT -> RED || DPAD RIGHT -> BLUE")
            .addLine("DPAD UP/DOWN -> CHANGE ROUTINE")
            .addLine()
            .addData("Current routine", routineType())
            .addData("Alliance", Poses.getCurrentAlliance())
            .update();
    }

    @Override
    protected void onInit() {
        Poses.setCurrentAlliance(currentAlliance);
        intake = new Intake(hardwareMap);
        intake.rampUp();

        shooter = new Shooter(hardwareMap);
    }

    @Override
    protected void onLoop() {
        intake.update();
        shooter.update();
        updateShooting();
    }

    private enum ShotState {IDLE, SPINNING_UP, TRANSFERING, DONE}
    private ShotState shotState = ShotState.IDLE;
    private final ElapsedTime transferTimer = new ElapsedTime();
    @Override
    protected void beginLeg() {
        shotState = ShotState.IDLE;
        intake.stop();
        shooter.stop();
        shotState = ShotState.IDLE;
        super.beginLeg();
    }
    protected void shoot() {
        shotState = ShotState.SPINNING_UP;
        transferTimer.reset();
    }
    protected boolean doneShooting() {
        return shotState == ShotState.DONE;
    }
    protected boolean outOfBalls() {
        return !intake.transferHasBall();
    }
    private void updateShooting() {
        if (shotState == ShotState.IDLE || shotState == ShotState.DONE) return;

        shooter.setTargets(currentPose, targetPose, ShooterUtil.DY, 0, 0, HOOD_OFFSET, TURRET_OFFSET);
        switch (shotState) {
            case SPINNING_UP:
                if (shooter.isReady() && transferTimer.seconds() > 1.0) {
                    intake.slowTransfer();
                    shotState = ShotState.TRANSFERING;
                    transferTimer.reset();
                }
                break;
            case TRANSFERING:
                if (!shooter.isReady()) intake.intake();
                else intake.slowTransfer();

                if (outOfBalls()) {
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
