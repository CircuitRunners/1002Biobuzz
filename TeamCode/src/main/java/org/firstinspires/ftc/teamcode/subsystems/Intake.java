package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intakeMotor;
    private enum IntakeState {IDLE, INTAKING, OUTTAKING}
    private IntakeState state;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        state = IntakeState.IDLE;
    }

    public void update() {
        switch (state) {
            case IDLE:
                intakeMotor.setPower(0);
                break;

            case INTAKING:
                intakeMotor.setPower(1.0);
                break;

            case OUTTAKING:
                intakeMotor.setPower(-1.0);
                break;

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
