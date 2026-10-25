package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;
public class Logger {

    private static final ElapsedTime timer = new ElapsedTime();
    private static String tag = "Logger";
    private static boolean enabled = true;
    private static boolean started = false;
    public static void start(OpMode opMode) {
        tag = opMode == null ? "Logger" : opMode.getClass().getSimpleName();
        enabled = true;
        started = true;
        timer.reset();
        write("START", tag);
    }

    public static void log(String name, Object value) {
        if (!started) throw new IllegalStateException("Call Logger.start() before logging");
        if (!enabled) return;
        write(name, format(value));
    }

    public static void log(String message) {
        if (!started) throw new IllegalStateException("Call Logger.start() before logging");
        if (!enabled) return;
        write("msg", message);
    }

    public static void setEnabled(boolean value) {
        enabled = value;
    }
    public static double seconds() {
        return timer.seconds();
    }

    private static void write(String name, String value) {
        RobotLog.ii(tag, "%.3f,%s,%s", timer.seconds(), escape(name), escape(value));
    }

    private static String format(Object value) {
        if (value instanceof Double || value instanceof Float) {
            return String.format(Locale.US, "%.4f", ((Number) value).doubleValue());
        }
        return String.valueOf(value);
    }
    private static String escape(String value) {
        return value.indexOf(',') < 0 ? value : value.replace(',', ';');
    }
}
