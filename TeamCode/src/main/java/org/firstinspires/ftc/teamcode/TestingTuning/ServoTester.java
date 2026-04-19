package org.firstinspires.ftc.teamcode.TestingTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo tester", group = "Test")
public class ServoTester extends OpMode {

    Servo hood;
    Servo blocker;

    double hoodPosition = 0.5;
    double blockerPosition = 0.8;

    double step_size = 0.025;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
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
            blockerPosition += step_size;
            if(blockerPosition > 1.0) {
                blockerPosition = 1.0;
            }
        }

        if(gamepad1.dpadLeftWasPressed()) {
            blockerPosition -= step_size;
            if(blockerPosition < 0.0) {
                blockerPosition = 0.0;
            }
        }

        hood.setPosition(hoodPosition);
        blocker.setPosition(blockerPosition);

        telemetry.addData("hood position", hoodPosition);
        telemetry.addData("blocker position", blockerPosition);
        telemetry.update();
    }
}
