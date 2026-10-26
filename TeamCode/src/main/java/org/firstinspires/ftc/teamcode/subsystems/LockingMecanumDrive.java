package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LockingMecanumDrive extends MecanumDrive {

    private final Servo lockingServo;
    public static double LOCK_POSITION = 0.34;
    public static double UNLOCK_POSITION = 1;
    public enum LMecDriveState {LOCKED, UNLOCKED}
    private LMecDriveState state;
    public LockingMecanumDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
        state = LMecDriveState.UNLOCKED;
        lockingServo = hardwareMap.get(Servo.class, "lockingServo");
        lockingServo.setPosition(UNLOCK_POSITION);
    }


    public void drive(double forward, double right, double rotate) {
        switch (state) {
            case LOCKED:
                super.drive(forward, 0, rotate);

                break;

            case UNLOCKED:
                super.drive(forward, right, rotate);
                break;
        }
    }

    public void driveFieldCentric(double forward, double right, double rotate, double h) {
        double theta = Math.atan2(forward, right) - h;
        double r = Math.hypot(forward, right);
        switch (state) {
            case LOCKED:
                drive(r * Math.sin(theta), 0, rotate);
                break;

            case UNLOCKED:
                drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
                break;
        }
    }

    public void lock() {
        lockingServo.setPosition(LOCK_POSITION);
        state = LMecDriveState.LOCKED;
    }

    public void unlock() {
        lockingServo.setPosition(UNLOCK_POSITION);
        state = LMecDriveState.UNLOCKED;
    }

    public LMecDriveState getState() {
        return state;
    }
}
