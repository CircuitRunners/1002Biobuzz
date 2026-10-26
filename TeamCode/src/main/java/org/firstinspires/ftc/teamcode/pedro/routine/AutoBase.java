package org.firstinspires.ftc.teamcode.pedro.routine;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Poses;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

public abstract class AutoBase extends OpMode {
    protected Follower follower;

    private static final PoseFactory MIRROR = PoseFactory.radians().mirrorX(72);

    private Routine routine;
    private int index;
    private boolean arrived;
    private double arrivalTime;
    private Object lastKey;
    protected GlobalTelemetry tele;
    protected Pose currentPose;
    protected Pose targetPose;
    protected Poses.Zone currentZone;

    private final Set<String> firedTCallbacks = new HashSet<>();
    private final Set<String> firedTimeCallbacks = new HashSet<>();
    private final Set<String> firedPoseCallbacks = new HashSet<>();

    protected abstract Pose startPose();
    protected abstract Routine routine();

    protected void onInit() {}
    protected boolean mirrored() { return false; }
    protected void initLoop() {}
    protected void onLoop() {}
    protected Object selectionKey() { return null; }
    protected boolean useLogging() { return false; }

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, useLogging());
        tele.addLine("Initializing...");
        follower = Constants.create(hardwareMap);
        onInit();
        rebuild();
        tele.addLine("Done!").update();
    }

    @Override
    public void init_loop() {
        initLoop();
        if (!Objects.equals(lastKey, selectionKey())) {
            tele.clearAll();
            tele.addLine("Building...");
            rebuild();
            tele.addLine("Done!").update();
        }
    }

    private void rebuild() {
        lastKey = selectionKey();

        Pose start = startPose();
        follower.setPose(start);
        routine = routine();
        routine.materializePaths(mirrored());

        firedTCallbacks.clear();
        firedTimeCallbacks.clear();
        firedPoseCallbacks.clear();
    }

    @Override
    public void start() {
        index = 0;
        beginLeg();
    }

    @Override
    public void stop() {
        Poses.TeleOpPoses.setTeleOpStartingPose(currentPose);
    }

    @Override
    public void loop() {
        follower.update();
        currentPose = follower.pose();
        currentZone = Poses.currentZone(currentPose);
        targetPose = Poses.targetHivePose(currentZone);
        onLoop();

        if (index >= routine.size()) {
            tele.addData("state", "done")
                .addLine(follower.debug().toString())
                .update();
            requestOpModeStop();
            return;
        }

        Routine.Leg leg = routine.leg(index);

        runParametricCallbacks(leg);
        runTemporalCallbacks(leg);
        runPoseCallbacks(leg);

        if (!arrived && !follower.isBusy()) {
            arrived = true;
            arrivalTime = getRuntime();
            if (leg.onArrival != null) {
                leg.onArrival.run();
            }
        }

        if (arrived && (getRuntime() - arrivalTime >= leg.pause) && ready(leg)) {
            index++;
            beginLeg();
        }

        tele.addData("Leg", index + "/" + routine.size())
            .addData("Held?", arrived && !ready(leg))
            .addData("T Value", follower.parametricCompletion())
            .addData("Pose", follower.pose())
            .update();
    }

    private void runParametricCallbacks(Routine.Leg leg) {
        double t = follower.parametricCompletion();
        leg.parametricTriggers.forEach((target, action) -> {
            String key = index + ":t:" + target;
            if (!firedTCallbacks.contains(key) && t >= target) {
                firedTCallbacks.add(key);
                action.run();
            }
        });
    }

    private void runTemporalCallbacks(Routine.Leg leg) {
        double elapsed = getRuntime() - arrivalTime;
        leg.temporalTriggers.forEach((ms, action) -> {
            String key = index + ":time:" + ms;
            if (!firedTimeCallbacks.contains(key) && elapsed * 1000.0 >= ms) {
                firedTimeCallbacks.add(key);
                action.run();
            }
        });
    }

    private void runPoseCallbacks(Routine.Leg leg) {
        Pose pose = follower.pose();
        leg.poseTriggers.forEach((target, action) -> {
            String key = index + ":pose:" + target;
            if (!firedPoseCallbacks.contains(key) && pose.distance(target) <= 0.5) {
                firedPoseCallbacks.add(key);
                action.run();
            }
        });
    }

    private boolean ready(Routine.Leg leg) {
        if (leg.waitUntil == null) return true;
        if ((getRuntime() - arrivalTime) >= leg.waitTimeout) return true;
        return leg.waitUntil.getAsBoolean();
    }

    protected void beginLeg() {
        arrived = false;
        arrivalTime = getRuntime();

        if (index < routine.size()) {
            Routine.Leg leg = routine.leg(index);
            if (leg.onStart != null) {
                leg.onStart.run();
            }
            if (leg.path != null) {
                follower.follow(leg.path);
            }
        }
    }

    static Pose mirror(Pose pose) {
        return MIRROR.of(pose.x(), pose.y(), pose.heading());
    }

    static ArrayList<Pose> mirror(ArrayList<Pose> poses) {
        ArrayList<Pose> mirroredPoses = new ArrayList<>();
        poses.forEach((p) -> mirroredPoses.add(mirror(p)));
        return mirroredPoses;
    }
}