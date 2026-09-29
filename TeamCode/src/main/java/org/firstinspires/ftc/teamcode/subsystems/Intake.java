package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final DcMotorEx intake1;
    private final DcMotorEx intake2;
    private final Servo transfer;
    private final double DISENGAGE_POSITION = 0.0;
    private final double ENGAGE_POSITION = 1.0;
    public enum IntakeState {IDLE, INTAKING, OUTTAKING}
    private IntakeState state;
    private double currentPower;
    private double targetPower;
    public Intake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(Servo.class, "transfer");
        disengageTransfer();

        state = IntakeState.IDLE;
    }

    public void update() {
        switch (state) {
            case IDLE:
                targetPower = 0.0;
                break;

            case INTAKING:
                targetPower = 1.0;
                break;

            case OUTTAKING:
                targetPower = -1.0;
                break;
        }

        if (currentPower != targetPower) {
            intake1.setPower(targetPower);
            intake2.setPower(targetPower);
            currentPower = targetPower;
        }
    }
    public void stop() {
        state = IntakeState.IDLE;
    }
    public void intake() {
        state = IntakeState.INTAKING;
    }
    public void outtake() {
        state = IntakeState.OUTTAKING;
    }
    public IntakeState getState() {
        return state;
    }
    public void engageTransfer() {
        transfer.setPosition(ENGAGE_POSITION);
    }
    public void disengageTransfer() {
        transfer.setPosition(DISENGAGE_POSITION);
    }
}
