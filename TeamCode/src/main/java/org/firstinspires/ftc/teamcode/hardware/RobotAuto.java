package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

public class RobotAuto extends Robot{

    public DrivetrainAuto drivetrainAuto;
    public RobotAuto(HardwareMap hardwareMap, Alliance alliance, Pose2D startPose) {
        super(hardwareMap, alliance);
        drivetrainAuto = new DrivetrainAuto(hardwareMap,startPose);
        drivetrainAuto.setTargetPose((alliance==Alliance.RED) ? PoseStorage.targetPoseRed :PoseStorage.targetPoseBlue);
    }

    public void update() {
        drivetrainAuto.update();
        super.update(drivetrainAuto.getPose());
        Pose2D limelightPose = getLimelightPose(drivetrainAuto.getVelocityX(), drivetrainAuto.getVelocityY(), drivetrainAuto.getAngularVelocity(), false);
        if (isUsingLimelight()) {
            drivetrainAuto.overridePose(limelightPose);
        }
        shooter.update(drivetrainAuto.getDistance(), true);
    }
}
