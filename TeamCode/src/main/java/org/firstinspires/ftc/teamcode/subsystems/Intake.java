package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private final DcMotorEx intake;
    private final DcMotorEx transfer;
    private final Servo blocker1;

    public static double UNBLOCK_POSITION = 0.88;
    public static double BLOCK_POSITION = 0.58;
    private final Servo ramp;
    public static double RAMP_DOWN = 0.67;
    public static double RAMP_UP = 1.0;
    private final DigitalChannel[] sensors = new DigitalChannel[5];
    public enum IntakeState {IDLE, INTAKING, OUTTAKING, TRANSFERING}
    private IntakeState state;
    private double currentIntakePower;
    private double targetIntakePower;
    private double currentTransferPower;
    private double targetTransferPower;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        blocker1 = hardwareMap.get(Servo.class, "blocker");

        unblockTransfer();

        ramp = hardwareMap.get(Servo.class, "flower");
        ramp.setDirection(Servo.Direction.REVERSE);
        rampUp();

        sensors[0] = hardwareMap.get(DigitalChannel.class, "sensor1");
        sensors[1] = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensors[2] = hardwareMap.get(DigitalChannel.class, "sensor3");
        sensors[3] = hardwareMap.get(DigitalChannel.class, "rampL");
        sensors[4] = hardwareMap.get(DigitalChannel.class, "rampR");

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
                targetTransferPower = transferHasBall() ? 0.0 : 1.0;
                break;

            case OUTTAKING:
                targetIntakePower = -1.0;
                targetTransferPower = -1.0;
                break;

            case TRANSFERING:
                targetIntakePower = 1.0;
                targetTransferPower = 0.5;
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
    }
    public void unblockTransfer() {
        blocker1.setPosition(UNBLOCK_POSITION);
    }
    public void rampUp() {ramp.setPosition(RAMP_UP);}
    public void rampDown() {ramp.setPosition(RAMP_DOWN);}
    public boolean rampHasBalls() {
        return sensors[3].getState() || sensors[4].getState();
    }
    public boolean transferHasBall() {return sensors[0].getState();}
}
