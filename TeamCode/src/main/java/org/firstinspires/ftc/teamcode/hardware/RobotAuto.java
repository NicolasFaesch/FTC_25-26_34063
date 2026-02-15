package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

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
                false
        );

         if (limelightPose != null) {
             drivetrainAuto.overridePose(limelightPose);
         }

        super.update(drivetrainAuto.getPose());
        shooter.update(drivetrainAuto.getDistance(), true);
    }

    public Pose2D getCurrentLimelightPose() {
        if (isUsingLimelight()) {
            return limelight.getPose(
                    drivetrainAuto.getVelocityX(),
                    drivetrainAuto.getVelocityY(),
                    drivetrainAuto.getAngularVelocity()
            );
        } else {
            return null;
        }
    }
}
