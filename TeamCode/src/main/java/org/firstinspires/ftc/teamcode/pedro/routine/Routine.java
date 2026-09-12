package org.firstinspires.ftc.teamcode.pedro.routine;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.routine.AutoBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Routine {
    enum Heading {
        LINEAR,
        CONSTANT,
        TANGENT,
        REVERSE_TANGENT,
        FACING_POINT,
        CUSTOM
    }

    static class Segment {
        final Pose end;
        final ArrayList<Pose> controlPoses = new ArrayList<>();
        Segment(Pose end, Pose... control) {
            this.end = end;
            Collections.addAll(controlPoses, control);
        }
    }

    static class Leg {
        final List<Segment> segments = new ArrayList<>();
        Heading heading = Heading.TANGENT;
        Interpolator customInterpolator = null;
        Pose facingPoint = null;

        boolean holdEnd = true;
        double pause = 0;
        Runnable onStart;
        Runnable onArrival;
        BooleanSupplier waitUntil;
        double waitTimeout = Double.POSITIVE_INFINITY;

        final Map<Double, Runnable> parametricTriggers = new LinkedHashMap<>();
        final Map<Double, Runnable> temporalTriggers = new LinkedHashMap<>();
        final Map<Pose, Runnable> poseTriggers = new LinkedHashMap<>();
        Path path;
    }

    private final Pose start;
    private final List<Leg> legs;

    private Routine(Pose start, List<Leg> legs) {
        this.start = start;
        this.legs = legs;
    }

    int size() {
        return legs.size();
    }

    Leg leg(int i) {
        return legs.get(i);
    }

    void materializePaths(boolean mirrored) {
        Pose cursor = mirrored ? AutoBase.mirror(start) : start;

        for (Leg leg : legs) {
            List<Path> built = new ArrayList<>();

            for (Segment s : leg.segments) {
                Pose end = mirrored ? AutoBase.mirror(s.end) : s.end;

                boolean controlled = !s.controlPoses.isEmpty();
                ArrayList<Pose> ctrlPoses = controlled
                        ? (mirrored ? AutoBase.mirror(s.controlPoses) : new ArrayList<>(s.controlPoses))
                        : null;

                if (controlled) {
                    ctrlPoses.add(0, cursor);
                    ctrlPoses.add(end);
                }

                Path path = controlled
                        ? Paths.curve(ctrlPoses.toArray(new Pose[0]))
                        : Paths.line(cursor, end);

                switch (leg.heading) {
                    case LINEAR:
                        path = path.linear(cursor, end);
                        break;
                    case CONSTANT:
                        path = path.constant(end);
                        break;
                    case TANGENT:
                        path = path.tangent();
                        break;
                    case REVERSE_TANGENT:
                        path = path.reverseTangent();
                        break;
                    case FACING_POINT:
                        path = path.facingPoint(leg.facingPoint == null ? end : (mirrored ? AutoBase.mirror(leg.facingPoint) : leg.facingPoint));
                        break;
                    case CUSTOM:
                        if (leg.customInterpolator != null) {
                            path = path.heading(leg.customInterpolator);
                        }
                        break;
                }

                built.add(path);
                cursor = end;
            }

            if (built.isEmpty()) {
                throw new IllegalStateException("A leg must contain at least one segment.");
            }

            leg.path = built.size() == 1
                    ? built.get(0)
                    : Paths.path(built.toArray(new Path[0]));
        }
    }

    public static Builder from(Pose start) {
        return new Builder(start);
    }

    public static class Builder {
        private final Pose start;
        private final List<Leg> legs = new ArrayList<>();
        private final List<Pose> pending = new ArrayList<>();

        private Builder(Pose start) {
            this.start = start;
        }

        public Builder through(Pose via) {
            pending.add(via);
            return this;
        }

        public Builder to(Pose end) {
            Leg leg = new Leg();
            for (Pose p : pending) {
                leg.segments.add(new Segment(p));
            }
            leg.segments.add(new Segment(end));
            pending.clear();
            legs.add(leg);
            return this;
        }

        public Builder curveTo(Pose end, Pose... controlPoses) {
            Leg leg = new Leg();
            for (Pose p : pending) {
                leg.segments.add(new Segment(p));
            }
            pending.clear();
            leg.segments.add(new Segment(end, controlPoses));
            legs.add(leg);
            return this;
        }

        public Builder interpolatedHeading() {
            last().heading = Heading.LINEAR;
            last().customInterpolator = null;
            return this;
        }

        public Builder constantHeading() {
            last().heading = Heading.CONSTANT;
            last().customInterpolator = null;
            return this;
        }

        public Builder tangentHeading() {
            last().heading = Heading.TANGENT;
            last().customInterpolator = null;
            return this;
        }

        public Builder reverseTangentHeading() {
            last().heading = Heading.REVERSE_TANGENT;
            last().customInterpolator = null;
            return this;
        }

        public Builder facingPoint(Pose point) {
            last().heading = Heading.FACING_POINT;
            last().facingPoint = point;
            return this;
        }

        public Builder customInterpolation(Interpolator interpolator) {
            last().heading = Heading.CUSTOM;
            last().customInterpolator = interpolator;
            return this;
        }

        public Builder noHold() {
            last().holdEnd = false;
            return this;
        }

        public Builder pause(double seconds) {
            last().pause = seconds;
            return this;
        }

        public Builder onStart(Runnable r) {
            last().onStart = r;
            return this;
        }

        public Builder onArrival(Runnable r) {
            last().onArrival = r;
            return this;
        }

        public Builder waitUntil(BooleanSupplier condition) {
            last().waitUntil = condition;
            return this;
        }

        public Builder waitUntil(BooleanSupplier condition, double timeoutSeconds) {
            last().waitUntil = condition;
            last().waitTimeout = timeoutSeconds;
            return this;
        }

        public Builder atTVal(double t, Runnable r) {
            if (last().parametricTriggers.containsKey(t)) {
                throw new IllegalStateException("Cannot run multiple Runnables for a singular t-value");
            }
            last().parametricTriggers.put(t, r);
            return this;
        }

        public Builder atTime(double seconds, Runnable r) {
            double key = seconds * 1000.0;
            if (last().temporalTriggers.containsKey(key)) {
                throw new IllegalStateException("Cannot run multiple Runnables for a singular time index");
            }
            last().temporalTriggers.put(key, r);
            return this;
        }

        public Builder atPose(Pose pose, Runnable r) {
            if (last().poseTriggers.containsKey(pose)) {
                throw new IllegalStateException("Cannot run multiple Runnables for a singular pose");
            }
            last().poseTriggers.put(pose, r);
            return this;
        }

        private Leg last() {
            if (legs.isEmpty()) {
                throw new IllegalStateException("modifier called before the first to()/curveTo()");
            }
            return legs.get(legs.size() - 1);
        }

        public Routine build() {
            if (!pending.isEmpty()) {
                throw new IllegalStateException("through() not followed by to()/curveTo()");
            }
            return new Routine(start, legs);
        }
    }
}