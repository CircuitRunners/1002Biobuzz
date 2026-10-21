package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.util.GlobalTelemetry;

@TeleOp(name = "DistanceSensorTester", group = "TESTING")
public class DistanceSensorTester extends OpMode {
    private GlobalTelemetry tele;
    private DigitalChannel sensor;
    private DigitalChannel[] sensors = new DigitalChannel[5];

    @Override
    public void init() {
        tele = new GlobalTelemetry(telemetry, false);
        tele.addLine("Initializing...").update();

        sensors[0] = hardwareMap.get(DigitalChannel.class, "sensor1");
        sensors[1] = hardwareMap.get(DigitalChannel.class, "sensor2");
        sensors[2] = hardwareMap.get(DigitalChannel.class, "sensor3");
        sensors[3] = hardwareMap.get(DigitalChannel.class, "rampL");
        sensors[4] = hardwareMap.get(DigitalChannel.class, "rampR");

        tele.addLine("Ready").update();
    }

    @Override
    public void loop() {
        tele.addData("Detecting?", sensor.getState()).update();
        // tele.addData("Num Balls", numBalls()).update();
    }

    private int numBalls() {
        int count = 0;
        for (int i = 0; i < 5; i++) {
            if (sensors[i].getState()) count++;
        }
        return count;
    }

}
