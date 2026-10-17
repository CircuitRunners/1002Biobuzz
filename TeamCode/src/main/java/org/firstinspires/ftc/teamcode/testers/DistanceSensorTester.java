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

        sensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
//        sensors[0] = hardwareMap.get(DigitalChannel.class, "bottomLeft");
//        sensors[1] = hardwareMap.get(DigitalChannel.class, "bottomRight");
//        sensors[2] = hardwareMap.get(DigitalChannel.class, "transfer3");
//        sensors[3] = hardwareMap.get(DigitalChannel.class, "transfer2");
//        sensors[4] = hardwareMap.get(DigitalChannel.class, "transfer1");

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
