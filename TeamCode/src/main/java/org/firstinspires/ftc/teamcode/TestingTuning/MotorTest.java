package org.firstinspires.ftc.teamcode.TestingTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo tester", group = "Test")
public class MotorTest extends OpMode {

    DcMotor motor1;
    DcMotor motor2;

    double motor1Power = 0.0;
    double motor2Power = 0.0;

    double step_size = 0.1;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor 1");
        motor2 = hardwareMap.get(DcMotor.class, "motor 2");
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasPressed()) {
            motor1Power += step_size;
            if(motor1Power > 1.0) {
                motor1Power = 1.0;
            }
        }

        if(gamepad1.dpadDownWasPressed()) {
            motor1Power -= step_size;
            if(motor1Power < -1.0) {
                motor1Power = 0.0;
            }
        }

        if(gamepad1.dpadRightWasPressed()) {
            motor2Power += step_size;
            if(motor2Power > 1.0) {
                motor2Power = 1.0;
            }
        }

        if(gamepad1.dpadLeftWasPressed()) {
            motor2Power -= step_size;
            if(motor2Power < -1.0) {
                motor2Power = 0.0;
            }
        }

        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);

        telemetry.addData("motor 1 power", motor1Power);
        telemetry.addData("motor 2 power", motor2Power);
        telemetry.update();
    }
}
