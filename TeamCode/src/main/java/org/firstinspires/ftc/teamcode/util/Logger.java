package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;


public class Logger {

    public static final File LOG_DIR = new File(AppUtil.FIRST_FOLDER, "logs");
    private static final int FLUSH_EVERY_ROWS = 50;

    private static final ElapsedTime timer = new ElapsedTime();
    private static final Map<String, String> row = new LinkedHashMap<>();
    private static final List<String> columns = new ArrayList<>();
    private static String tag = "Logger";
    private static boolean enabled = true;
    private static boolean started = false;
    private static BufferedWriter out;
    private static File file;
    private static int rowsSinceFlush = 0;

    public static void start(OpMode opMode) {
        stop(); // close a file left open by a previous OpMode
        tag = opMode == null ? "Logger" : opMode.getClass().getSimpleName();
        enabled = true;
        started = true;
        row.clear();
        columns.clear();
        rowsSinceFlush = 0;
        timer.reset();

        String stamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
        file = new File(LOG_DIR, tag + "_" + stamp + ".csv");
        try {
            if (!LOG_DIR.isDirectory() && !LOG_DIR.mkdirs()) throw new IOException("could not create " + LOG_DIR);
            out = new BufferedWriter(new FileWriter(file));
            RobotLog.ii(tag, "Logging to %s", file.getAbsolutePath());
        } catch (IOException e) {
            out = null;
            RobotLog.ee(tag, e, "Failed to open log file %s", file);
        }
    }

    /** Adds a value to the current row. Poses are split into X, Y and Heading (radians) columns. */
    public static void log(String name, Object value) {
        if (!started) throw new IllegalStateException("Call Logger.start() before logging");
        if (!enabled) return;
        if (value instanceof Pose) {
            Pose p = (Pose) value;
            row.put(name + " X", format(p.x()));
            row.put(name + " Y", format(p.y()));
            row.put(name + " Heading", format(p.heading()));
        } else {
            row.put(name, format(value));
        }
    }

    /** Free-text lines (telemetry headers, blank lines) are display-only and are not logged. */
    public static void log(String message) {
        if (!started) throw new IllegalStateException("Call Logger.start() before logging");
    }

    /** Writes the current row as one CSV line and clears it. Call once per loop. */
    public static void endLoop() {
        if (out == null || row.isEmpty()) {
            row.clear();
            return;
        }
        try {
            boolean newColumns = false;
            for (String key : row.keySet()) {
                if (!columns.contains(key)) {
                    columns.add(key);
                    newColumns = true;
                }
            }
            if (newColumns) {
                StringBuilder header = new StringBuilder("t");
                for (String c : columns) header.append(',').append(escape(c));
                out.write(header.toString());
                out.newLine();
            }

            StringBuilder line = new StringBuilder(String.format(Locale.US, "%.4f", timer.seconds()));
            for (String c : columns) {
                String v = row.get(c);
                line.append(',').append(v == null ? "" : v);
            }
            out.write(line.toString());
            out.newLine();

            if (++rowsSinceFlush >= FLUSH_EVERY_ROWS) {
                out.flush();
                rowsSinceFlush = 0;
            }
        } catch (IOException e) {
            RobotLog.ee(tag, e, "Failed to write log row; disabling file logging");
            closeQuietly();
        }
        row.clear();
    }

    public static void stop() {
        if (out != null) {
            if (!row.isEmpty()) endLoop();
            closeQuietly();
        }
        started = false;
    }

    public static void setEnabled(boolean value) {
        enabled = value;
    }

    public static double seconds() {
        return timer.seconds();
    }

    public static File getFile() {
        return file;
    }

    private static void closeQuietly() {
        try {
            if (out != null) out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }

    private static String format(Object value) {
        if (value instanceof Double || value instanceof Float) {
            return String.format(Locale.US, "%.4f", ((Number) value).doubleValue());
        }
        return escape(String.valueOf(value));
    }

    private static String escape(String value) {
        return value.indexOf(',') < 0 ? value : value.replace(',', ';');
    }
}
