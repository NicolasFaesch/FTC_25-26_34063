package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.lib.Drawing.drawDebug;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

import java.util.Locale;

public class RobotAuto extends Robot {

    public DrivetrainAuto drivetrainAuto;

    public RobotAuto(HardwareMap hardwareMap, Alliance alliance, Pose2D startPose) {
        super(hardwareMap, alliance);

        drivetrainAuto = new DrivetrainAuto(hardwareMap, startPose);
        setDrivetrain(drivetrainAuto);
    }

    public void update() {
        drivetrainAuto.update();
        super.update();
    }

    public void updateTelemetry(TelemetryManager panelsTelemetry, Telemetry telemetry) {
        // Position
        Pose2D botPose = drivetrainAuto.getPose();
        String position = String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.1f",
                botPose.getX(DistanceUnit.INCH),
                botPose.getY(DistanceUnit.INCH),
                botPose.getHeading(AngleUnit.DEGREES));


        // Robot States
        panelsTelemetry.addLine("=== ROBOT STATES ===");
        panelsTelemetry.addData("Robot State", getState());
        panelsTelemetry.addData("Intake State", intake.getState());
        panelsTelemetry.addData("Transfer State", transfer.getState());
        panelsTelemetry.addData("Shooter State", shooter.getState());


        telemetry.addLine("=== ROBOT STATES ===");
        telemetry.addData("Robot State", getState());
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Transfer State", transfer.getState());
        telemetry.addData("Shooter State", shooter.getState());

        // Pose
        panelsTelemetry.addLine("=== POSE ===");
        panelsTelemetry.addData("Fused", position);

        telemetry.addLine("=== POSE ===");
        telemetry.addData("Fused", position);

        // Shooter Info
        panelsTelemetry.addLine("=== SHOOTER ===");
        if (shooter.getManualOverride()) {
            panelsTelemetry.addLine("Shooter: MANUAL OVERRIDE");
            panelsTelemetry.addData("Hood Position (manual)", shooter.getHoodPositionManual());
            panelsTelemetry.addData("Target Velocity (manual)", shooter.getShooterTargetVelocityManual());
        } else {
            panelsTelemetry.addData("Hood Position", shooter.getHoodPosition());
            panelsTelemetry.addData("Target Velocity", shooter.getShooterTargetVelocity());
        }
        panelsTelemetry.addData("Current Velocity", shooter.getShooterVelocity());

        telemetry.addLine("=== SHOOTER ===");
        if (shooter.getManualOverride()) {
            telemetry.addLine("Shooter: MANUAL OVERRIDE");
            telemetry.addData("Hood Position (manual)", shooter.getHoodPositionManual());
            telemetry.addData("Target Velocity (manual)", shooter.getShooterTargetVelocityManual());
        } else {
            telemetry.addData("Hood Position", shooter.getHoodPosition());
            telemetry.addData("Target Velocity", shooter.getShooterTargetVelocity());
        }
        telemetry.addData("Current Velocity", shooter.getShooterVelocity());

        // Update PanelsTelemetry
        //drawPath();
        drawDebug(drivetrainAuto.getFollower());
        panelsTelemetry.update();
        telemetry.update();
    }
}
