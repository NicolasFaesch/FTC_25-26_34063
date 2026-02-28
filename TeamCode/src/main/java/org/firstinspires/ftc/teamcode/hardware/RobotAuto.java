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

        drivetrainAuto.setTargetPose(
                (alliance == Alliance.RED) ? PoseStorage.targetPoseRed : PoseStorage.targetPoseBlue
        );
    }

    public void update() {
        drivetrainAuto.update();

        Pose2D limelightPose = getLimelightPose(
                drivetrainAuto.getVelocityX(),
                drivetrainAuto.getVelocityY(),
                drivetrainAuto.getAngularVelocity(),
                true
        );

         if (limelightPose != null) {
             drivetrainAuto.overridePose(limelightPose);
         }

        super.update(drivetrainAuto.getPose());
        shooter.update(drivetrainAuto.getDistance(), true);
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
        if (!isUsingLimelight()) {
            panelsTelemetry.addData("Odometry", position);
        } else {
            panelsTelemetry.addData("Limelight", position);
        }
        panelsTelemetry.addData("Distance", drivetrainAuto.getDistance());

        telemetry.addLine("=== POSE ===");
        if (!isUsingLimelight()) {
            telemetry.addData("Odometry", position);
        } else {
            telemetry.addData("Limelight", position);
        }
        telemetry.addData("Distance", drivetrainAuto.getDistance());

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
