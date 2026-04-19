package org.firstinspires.ftc.teamcode.TestingTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.AxonEncoder;
import org.firstinspires.ftc.teamcode.hardware.Turret;

@TeleOp(name="Turret Test - Continuous Servo")
public class TurretTestContinuousServo extends OpMode {

    Turret turret;
    double targetAngle = 0;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        // set zeros when aligned to front
        if (gamepad1.leftStickButtonWasPressed() && gamepad1.rightStickButtonWasPressed()) {
            turret.setAxonEncoderZero();
            //TODO: implement
            //turret.setMagneticEncoderZero();
        }

        if (gamepad1.aWasPressed())
            turret.setState(Turret.State.TRACKING);

        if (gamepad1.xWasPressed())
            turret.setState(Turret.State.STORED);

        if (gamepad1.yWasPressed())
                turret.setState(Turret.State.IDLE);

        if (gamepad1.bWasPressed()) {
            turret.setManualOverride(!turret.getManualOverride());
           targetAngle = turret.getTurretAngle();
        }

        if(gamepad1.leftBumperWasPressed())
            turret.decreaseManualTurretAngle();

        if (gamepad1.rightBumperWasPressed())
            turret.increaseManualTurretAngle();


        if(Math.abs(gamepad1.left_trigger) > 0.1) {
            double leftStickX = gamepad1.left_stick_x;
            if(leftStickX > 0) {
                targetAngle = Turret.TURRET_MAX_ANGLE * leftStickX;
            } else {
                targetAngle = Turret.TURRET_MIN_ANGLE * -leftStickX;
            }
        }

        if(gamepad1.dpadDownWasPressed())
           targetAngle = Turret.TURRET_MAX_ANGLE;

        if(gamepad1.dpadUpWasPressed())
           targetAngle = Turret.TURRET_STORED_ANGLE;

        if(gamepad1.dpadRightWasPressed())
           targetAngle = 90;

        if (gamepad1.dpadLeftWasPressed())
           targetAngle = -90;

        turret.update(targetAngle);


        if(!turret.getManualOverride()) {
            telemetry.addData("Turret Target Angle", turret.getTargetAngle());
        } else {
            telemetry.addData("Turret Target Angle (manual)", turret.getTargetAngleManual());
        }
        telemetry.addData("Turret Angle", turret.getTurretAngle());
        telemetry.addData("Axon Raw Degrees", turret.getAxonRawDegrees());
        telemetry.addData("Axon Voltage", turret.getAxonVoltage());
        telemetry.addLine("");
        telemetry.addData("Turret State", turret.getState());
        telemetry.addData("Turret On Target", turret.isOnTarget());
        telemetry.addLine("");
        telemetry.addLine("---------");
        telemetry.addLine("");
        telemetry.addLine("To reset encoder zeroes, press both sticks simultaneously with turret facing front (0°)");
    }


}
