package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intakeMotor;
    public enum IntakeState {IDLE, INTAKING, OUTTAKING}
    private IntakeState state;
    private double currentPower;
    private double targetPower;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            intakeMotor.setPower(targetPower);
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
}
