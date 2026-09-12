package org.firstinspires.ftc.teamcode.pedro.routine;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        follower = Constants.create(hardwareMap);
        onInit();
        rebuild();
        telemetry.addLine("Done!");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        initLoop();
        if (!Objects.equals(lastKey, selectionKey())) {
            telemetry.clearAll();
            telemetry.addLine("Building...");
            rebuild();
            telemetry.addLine("Done!");
            telemetry.update();
        }
    }

    private void rebuild() {
        lastKey = selectionKey();

        Pose start = mirrored() ? mirror(startPose()) : startPose();
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
    public void loop() {
        follower.update();
        onLoop();

        if (index >= routine.size()) {
            telemetry.addData("state", "done");
            telemetry.addLine(follower.debug().toString());
            telemetry.update();
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

        telemetry.addData("leg", index + "/" + routine.size());
        telemetry.addData("held", arrived && !ready(leg));
        telemetry.addData("t", follower.parametricCompletion());
        telemetry.addData("pose", follower.pose());
        telemetry.addLine(follower.debug().toString());
        telemetry.update();
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

    private void beginLeg() {
        arrived = false;
        arrivalTime = getRuntime();

        if (index < routine.size()) {
            Routine.Leg leg = routine.leg(index);
            if (leg.onStart != null) {
                leg.onStart.run();
            }
            follower.follow(leg.path);
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