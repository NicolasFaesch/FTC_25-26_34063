package org.firstinspires.ftc.teamcode.TestingTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Controller;

@Configurable
@TeleOp(name="Shooter Test - Manual P-Control")
public class ShooterTest extends OpMode {
    // Hardware
    private DcMotorEx shooterMaster, shooterSlave;
    private Servo hood;

    // Tuning Constants
    // kV: Base power to reach RPM (Start around 0.00015)
    public static double kV = 0.000215;
    // kP: Correction strength (Start very small, e.g., 0.00005)
    public static double kP = 0.0005;

    private static final double MOTOR_CPR = 28.0;
    private static final double GEAR_RATIO = 1.0;

    // Control Variables
    private double targetRPM = 2500;
    private double targetHoodPos = 0.5;
    private boolean isShooterOn = false;

    @Override
    public void init() {
        // Assume 'shooterLeft' has the encoder cable
        shooterMaster = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterSlave = hardwareMap.get(DcMotorEx.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");

        shooterMaster.setDirection(DcMotorEx.Direction.FORWARD);
        shooterSlave.setDirection(DcMotorEx.Direction.REVERSE);

        // MUST be in RUN_WITHOUT_ENCODER to allow manual power control
        shooterMaster.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterSlave.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterMaster.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterSlave.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        // 1. Handle Inputs
        if (gamepad1.dpadUpWasPressed()) targetRPM += 100;
        if (gamepad1.dpadDownWasPressed()) targetRPM -= 100;
        if (gamepad1.dpadRightWasPressed()) targetHoodPos += 0.05;
        if (gamepad1.dpadLeftWasPressed()) targetHoodPos -= 0.05;

        if (gamepad1.a) isShooterOn = true;
        if (gamepad1.b) isShooterOn = false;

        // 2. Constraints
        targetRPM = Math.max(0, Math.min(targetRPM, 6000));
        targetHoodPos = Math.max(0.1, Math.min(targetHoodPos, 0.975));

        // 3. Control Logic
        if (isShooterOn) {
            double currentRPM = toRPM(shooterMaster.getVelocity());
            double error = targetRPM - currentRPM;

            // Calculate Power: Feedforward + Proportional Correction
            double ff = targetRPM * kV;
            double p_corr = error * kP;
            double totalPower = ff + p_corr;

            // Clip power to valid range [-1, 1]
            totalPower = Math.max(-1.0, Math.min(totalPower, 1.0));

            shooterMaster.setPower(totalPower);
            shooterSlave.setPower(totalPower);

            telemetry.addData("Error", error);
            telemetry.addData("P Correction", p_corr);
        } else {
            shooterMaster.setPower(0);
            shooterSlave.setPower(0);
        }

        hood.setPosition(targetHoodPos);

        // 4. Telemetry
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", toRPM(shooterMaster.getVelocity()));
        telemetry.addData("Hood Pos", targetHoodPos);
        telemetry.update();
    }

    private double toRPM(double tps) {
        return (tps / MOTOR_CPR) * 60.0 / GEAR_RATIO;
    }
}