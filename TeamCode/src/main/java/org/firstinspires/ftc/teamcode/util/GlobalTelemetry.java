package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Locale;

public class GlobalTelemetry {
    public static final int DEFAULT_DECIMALS = 5;
    private final Telemetry sdkTelemetry;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public GlobalTelemetry(Telemetry telemetry) {
        sdkTelemetry = telemetry;
        sdkTelemetry.setAutoClear(true);
        clearAll();
    }

    public GlobalTelemetry addData(String caption, Object value) {
        return write(caption, String.valueOf(value));
    }

    public GlobalTelemetry addData(String caption, double value) {
        return addData(caption, value, DEFAULT_DECIMALS);
    }

    public GlobalTelemetry addData(String caption, int value) {
        return write(caption, Integer.toString(value));
    }

    public GlobalTelemetry addData(String caption, long value) {
        return write(caption, Long.toString(value));
    }

    public GlobalTelemetry addData(String caption, double value, int decimals) {
        return write(caption, String.format(Locale.US, "%." + decimals + "f", value));
    }

    public GlobalTelemetry addData(String caption, String format, Object... args) {
        return write(caption, String.format(Locale.US, format, args));
    }
    public GlobalTelemetry addLine(String line) {
        sdkTelemetry.addLine(line);
        panelsTelemetry.addLine(line);
        return this;
    }

    public GlobalTelemetry addLine() {
        return addLine("");
    }
    public GlobalTelemetry header(String title) {
        return addLine("===== " + title + " =====");
    }
    public GlobalTelemetry panelsOnly(String caption, Object value) {
        panelsTelemetry.addData(caption, String.valueOf(value));
        return this;
    }
    public GlobalTelemetry stationOnly(String caption, Object value) {
        sdkTelemetry.addData(caption, String.valueOf(value));
        return this;
    }

    public void update() {
        sdkTelemetry.update();
        panelsTelemetry.update();
    }

    public void clearAll() {
        sdkTelemetry.clearAll();
        panelsTelemetry.setLines(new ArrayList<>());
    }
    public GlobalTelemetry setStationInterval(int ms) {
        sdkTelemetry.setMsTransmissionInterval(ms);
        return this;
    }
    public Telemetry station() {
        return sdkTelemetry;
    }
    public TelemetryManager panels() {
        return panelsTelemetry;
    }

    private GlobalTelemetry write(String caption, String value) {
        sdkTelemetry.addData(caption, value);
        panelsTelemetry.addData(caption, value);
        return this;
    }
}
