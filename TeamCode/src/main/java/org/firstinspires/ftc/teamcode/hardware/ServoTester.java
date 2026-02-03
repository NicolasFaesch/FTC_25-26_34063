package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.jar.Attributes;

@TeleOp(name = "Servo tester", group = "Test")
public class ServoTester extends OpMode {

    Servo hood;
    Servo feeder;

    double hoodPosition = 0.5;
    double feederPosition = 0.8;

    double step_size = 0.025;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        feeder = hardwareMap.get(Servo.class, "feeder_servo");
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) {
            hoodPosition += step_size;
            if(hoodPosition > 1.0) {
                hoodPosition = 1.0;
            }
        }

        if(gamepad1.dpadDownWasPressed()) {
            hoodPosition -= step_size;
            if(hoodPosition < 0.0) {
                hoodPosition = 0.0;
            }
        }

        if(gamepad1.dpadRightWasPressed()) {
            feederPosition += step_size;
            if(feederPosition > 1.0) {
                feederPosition = 1.0;
            }
        }

        if(gamepad1.dpadLeftWasPressed()) {
            feederPosition -= step_size;
            if(feederPosition < 0.0) {
                feederPosition = 0.0;
            }
        }

        hood.setPosition(hoodPosition);
        feeder.setPosition(feederPosition);

        telemetry.addData("hood position", hoodPosition);
        telemetry.addData("feeder position", feederPosition);
        telemetry.update();
    }
}
