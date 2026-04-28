package org.firstinspires.ftc.teamcode.TestingTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.AS5600Driver;

@TeleOp(name = "AS5600 Test")
public class AS5600Test extends OpMode {
    AS5600Driver magneticEncoder;

    @Override
    public void init() {
        magneticEncoder = hardwareMap.get(AS5600Driver.class, "magnetic encoder");
    }

    @Override
    public void init_loop() {
        String magnetStatus = magneticEncoder.getMagnetStatus();
        telemetry.addData("Magnet Status", magnetStatus);
        if (magnetStatus.equals("READY")) {
            double agc = magneticEncoder.getAgcValue();
            telemetry.addData("AGC value (0-255) Target 128", agc);
        }
        telemetry.update();

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            magneticEncoder.setZero();
        }
        telemetry.addData("Raw Degrees", magneticEncoder.getRawDegrees());
        telemetry.addData("Angle", magneticEncoder.getAngle());
        telemetry.addData("Zero offset", magneticEncoder.getOffsetDegrees());
        telemetry.update();
    }
}
