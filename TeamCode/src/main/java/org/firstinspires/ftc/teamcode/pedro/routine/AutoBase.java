package org.firstinspires.ftc.teamcode.pedro.routine;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.CheckedOctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.util.Poses;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

public abstract class AutoBase extends OpMode {
    protected Follower follower;

    private static final PoseFactory MIRROR = PoseFactory.radians().mirrorX(72);

    private Routine routine;
    private int index;
    private boolean arrived;
    private static final double DRIVE_TIMEOUT_SECONDS = 10.0;
    private double legStartTime;
    private double arrivalTime;
    private Object lastKey;
    protected GlobalTelemetry tele;
    protected Pose currentPose;
    protected Pose targetPose;
    protected Poses.Zone currentZone;

    private final Set<String> firedTCallbacks = new HashSet<>();
    private final Set<String> firedTimeCallbacks = new HashSet<>();
    private final Set<String> firedPoseCallbacks = new HashSet<>();
    private final Deque<RoutineFrame> routineStack = new ArrayDeque<>();

    protected abstract Pose startPose();
    protected abstract Routine routine();

    protected void onInit() {}
    protected void initLoop() {}
    protected void onLoop() {}
    protected Object selectionKey() { return null; }
    protected boolean useLogging() { return false; }
    private static class RoutineFrame {
        final Routine routine;
        final int nextIndex;

        RoutineFrame(Routine routine, int nextIndex) {
            this.routine = routine;
            this.nextIndex = nextIndex;
        }
    }

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, useLogging());
        tele.addLine("Initializing...");
        follower = Constants.create(hardwareMap);
        onInit();
        rebuild();
        tele.addLine("Routine built").update();
    }

    @Override
    public void init_loop() {
        initLoop();
        if (!Objects.equals(lastKey, selectionKey())) {
            tele.clearAll();
            tele.addLine("Building...");
            rebuild();
            tele.addLine("Routine built").update();
        }
        Constants.localizationReady(follower);
        tele.addData("Odometry", Constants.localizationStatus(follower)).addData("Running routine", lastKey).update();
    }

    private void rebuild() {
        lastKey = selectionKey();

        Pose start = startPose();
        follower.setPose(start);
        routine = routine();
        routine.materializePaths(null);

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
        if (currentPose != null) Poses.TeleOpPoses.setTeleOpStartingPose(currentPose);
    }

    @Override
    public void loop() {
        try {
            follower.update();
        } catch (CheckedOctoQuadLocalizer.LocalizationUnavailable e) {
            abortRoutine(e.getMessage());
            return;
        }
        currentPose = follower.pose();
        currentZone = Poses.currentZone(currentPose);
        targetPose = Poses.targetHivePose(currentZone);
        onLoop();

        if (index >= routine.size()) {
            if (routineStack.isEmpty()) {
                requestOpModeStop();
            } else {    // finish current routine
                returnFromBranch();
            }
            return;
        }

        Routine.Leg leg = routine.leg(index);

        if (!arrived && leg.path != null && follower.isBusy() && getRuntime() - legStartTime >= DRIVE_TIMEOUT_SECONDS) {
            abortRoutine("Drive timed out on leg " + index);
            return;
        }
        runParametricCallbacks(leg);
        runTemporalCallbacks(leg);
        runPoseCallbacks(leg);

        if ((!arrived && !follower.isBusy())) {
            arrived = true;
            arrivalTime = getRuntime();
            if (leg.onArrival != null) leg.onArrival.run();
            if (leg.holdEnd) follower.hold(currentPose);
        }

        if (arrived && (getRuntime() - arrivalTime >= leg.pause) && ready(leg)) {
            if (leg.decision != null) { // a decision was made so we need to jump to a different branch
                Routine selectedBranch = leg.decision.getAsBoolean()? leg.trueBranch : leg.falseBranch;
                enterBranch(selectedBranch);
                return;
            }

            // otherwise, if no decision was made, continue to the next leg normally
            index++;
            beginLeg();
        }

        tele.addData("Running routine", lastKey)
            .addData("Leg", index + "/" + routine.size())
            .addData("Stack depth", routineStack.size())
            .addData("Held?", arrived && !ready(leg))
            .addData("T Value", follower.parametricCompletion())
            .addData("Pose", follower.pose())
            .addData("X Velo", follower.velocity().vx)
            .addData("Y Velo", follower.velocity().vy)
            .update();
    }

    private void abortRoutine(String reason) {
        follower.stop();
        com.qualcomm.robotcore.util.RobotLog.addGlobalWarningMessage("Auto stopped: " + reason);
        tele.addData("Auto stopped", reason).update();
        requestOpModeStop();
    }

    private void enterBranch(Routine branch) {
        if (branch == null) {
            throw new IllegalArgumentException("Cannot enter a null branch.");
        }

        // save the parent routine so we can come back to it
        RoutineFrame frame = new RoutineFrame(routine, index + 1);
        routineStack.push(frame);

        // now build the new branch from the robot's current pose
        branch.materializePaths(currentPose);
        routine = branch;       // set current routine to this branch
        index = 0;      // index of this current branch

        arrived = false;
        firedTCallbacks.clear();
        firedTimeCallbacks.clear();
        firedPoseCallbacks.clear();
        start();
    }
    private void returnFromBranch() {
        // go back to the parent routine
        RoutineFrame frame = routineStack.pop();
        routine = frame.routine;
        index = frame.nextIndex;    // index to pick up from

        // rebuild the remaining paths
        routine.materializePaths(currentPose, index);

        arrived = false;
        firedTCallbacks.clear();
        firedTimeCallbacks.clear();
        firedPoseCallbacks.clear();

        beginLeg();
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
        double elapsed = getRuntime() - legStartTime;
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
        if ((getRuntime() - arrivalTime) >= leg.pause + leg.waitTimeout) return true;
        return leg.waitUntil.getAsBoolean();
    }

    protected void beginLeg() {
        arrived = false;
        legStartTime = getRuntime();
        arrivalTime = legStartTime;

        if (index < routine.size()) {
            Routine.Leg leg = routine.leg(index);
            if (leg.onStart != null) leg.onStart.run();
            if (leg.path != null) follower.follow(leg.path);
        }
    }
}