package org.firstinspires.ftc.teamcode.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Poses;
import org.firstinspires.ftc.teamcode.util.ShooterUtil;

import java.util.List;

@Configurable
@TeleOp(name = "TurretTester", group = "TESTING")
public class TurretTester extends OpMode {

    private GlobalTelemetry tele;
    private Shooter.Turret turret;
    private Follower follower;
    private LockingMecanumDrive lMec;
    private List<LynxModule> hubs;
    private GamepadEx player1;
    private double position = 0.5;
    public static double angle = 0.0;
    public static boolean angleToggle = false;
    public static double offset = 0.0;

    @Override
    public void init() {
        Logger.start(this);
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing...")
                .update();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        lMec = new LockingMecanumDrive(hardwareMap);
        turret = new Shooter.Turret(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.relocPose1);

        player1 = new GamepadEx(gamepad1);

        tele.addLine("Ready")
                .update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();

        player1.readButtons();
        follower.update();
        Pose currentPose = follower.pose();
        Poses.Zones currentZone = Poses.currentZone(currentPose);
        Pose targetPose = Poses.targetHivePose(currentZone);

        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) position += 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) position -= 0.01;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) position += 0.001;
        if (player1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) position -= 0.001;
        position = Range.clip(position, 0, 1);

        if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) angleToggle = !angleToggle;
        if (!angleToggle) turret.setPosition(position);
        else turret.setTarget(currentPose, targetPose, offset);

        if (player1.wasJustPressed(GamepadKeys.Button.SQUARE)) follower.setPose(Poses.relocPose1);
        double forward = player1.getLeftY();
        double strafe = player1.getLeftX();
        double rotate = player1.getRightX();
        lMec.driveFieldCentric(-forward, -strafe, rotate, currentPose.heading());

        tele.addData("Current Position", position)
                .addData("Calculated Angle", turret.getAngle())
                .addData("Current Follower Pose", currentPose)
                .showLoopTime()
                .update();
    }
}
