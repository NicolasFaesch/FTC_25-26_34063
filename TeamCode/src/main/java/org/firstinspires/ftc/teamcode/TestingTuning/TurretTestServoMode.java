package org.firstinspires.ftc.teamcode.TestingTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Configurable
@TeleOp(name="Turret Test - Servo Mode")
public class TurretTestServoMode extends OpMode {
    Servo turretServoLeft;
    Servo turretServoRight;

    double turretServoPosition;
    public  double stepSize = 0.0025;
    public static double turretBackPosition = 1.0;
    public static double turretFrontPosition = 0.47;
    public static double turretLeftPosition = 0.2;
    public static double turretRightPosition = 0.75;

    public static double leftServoMismatch = 0.008;

    public static final double SERVO_MIN_POSITION = 0.0;
    public static final double SERVO_MAX_POSITION = 1.0;

    @Override
    public void init() {
        turretServoLeft = hardwareMap.get(Servo.class, "TurretServoLeft");
        turretServoRight = hardwareMap.get(Servo.class, "TurretServoRight");

        turretServoPosition = turretFrontPosition;
        updateServos(turretServoPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            updateServos(turretFrontPosition);
        } else if (gamepad1.dpad_down) {
            updateServos(turretBackPosition + leftServoMismatch);
        } else if (gamepad1.dpad_left) {
            updateServos(turretLeftPosition);
        } else if (gamepad1.dpad_right) {
            updateServos(turretRightPosition);
        } else if (gamepad1.right_bumper && turretServoPosition < SERVO_MAX_POSITION) {
            updateServos(turretServoPosition + stepSize);
        } else if (gamepad1.left_bumper && turretServoPosition > SERVO_MIN_POSITION) {
            updateServos(turretServoPosition - stepSize);
        }

        telemetry.addData("Turret Servo Position", turretServoPosition);
    }

    public void updateServos(double ServoTargetPosition) {
        turretServoLeft.setPosition(ServoTargetPosition - leftServoMismatch);
        turretServoRight.setPosition(ServoTargetPosition);
        turretServoPosition = ServoTargetPosition;
    }
}
