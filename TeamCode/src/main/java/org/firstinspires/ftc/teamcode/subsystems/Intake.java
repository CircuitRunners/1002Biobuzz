package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final DcMotorEx intake;
    private final DcMotorEx transfer;
    private final Servo blocker1;
    private final Servo blocker2;
    public static double UNBLOCK_POSITION = 0.0;
    public static double BLOCK_POSITION = 1.0;
    private final Servo ramp;
    public static double RAMP_DOWN = 0.0;
    public static double RAMP_UP = 1.0;
    public enum IntakeState {IDLE, INTAKING, OUTTAKING, TRANSFERING}
    private IntakeState state;
    private double currentIntakePower;
    private double targetIntakePower;
    private double currentTransferPower;
    private double targetTransferPower;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake1");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotorEx.class, "intake2");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        blocker1 = hardwareMap.get(Servo.class, "blocker1");
        blocker2 = hardwareMap.get(Servo.class, "blocker2");
        blocker2.setDirection(Servo.Direction.REVERSE);
        unblockTransfer();

        ramp = hardwareMap.get(Servo.class, "ramp");

        state = IntakeState.IDLE;
    }

    public void update() {
        switch (state) {
            case IDLE:
                targetIntakePower = 0.0;
                targetTransferPower = 0.0;
                break;

            case INTAKING:
                targetIntakePower = 1.0;
                break;

            case OUTTAKING:
                targetIntakePower = -1.0;
                targetTransferPower = -1.0;
                break;

            case TRANSFERING:
                targetIntakePower = 1.0;
                targetTransferPower = 1.0;
        }

        if (currentIntakePower != targetIntakePower) {
            intake.setPower(targetIntakePower);
            currentIntakePower = targetIntakePower;
        }

        if (currentTransferPower != targetTransferPower) {
            transfer.setPower(targetTransferPower);
            currentTransferPower = targetTransferPower;
        }
    }
    public void stop() {
        state = IntakeState.IDLE;
    }
    public void intake() {
        state = IntakeState.INTAKING;
        blockTransfer();
    }
    public void outtake() {
        state = IntakeState.OUTTAKING;
    }
    public void transfer() {
        state = IntakeState.TRANSFERING;
        unblockTransfer();
    }
    public IntakeState getState() {
        return state;
    }
    public void blockTransfer() {
        blocker1.setPosition(BLOCK_POSITION);
        blocker2.setPosition(BLOCK_POSITION);
    }
    public void unblockTransfer() {
        blocker1.setPosition(UNBLOCK_POSITION);
        blocker2.setPosition(UNBLOCK_POSITION);
    }
    public void rampUp() {ramp.setPosition(RAMP_UP);}
    public void rampDown() {ramp.setPosition(RAMP_DOWN);}
}
