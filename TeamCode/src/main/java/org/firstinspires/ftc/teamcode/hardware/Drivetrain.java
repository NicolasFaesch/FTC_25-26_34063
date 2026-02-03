package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.math.Vector;

public class Drivetrain {
    protected Follower follower;

    protected Pose targetPose;

    protected Drivetrain(HardwareMap hardwareMap, Pose2D startPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pose2DtoPose(startPose));
    }

    protected void update() {
        follower.update();
        PoseStorage.currentPose = getPose();
    }

    public void overridePose(Pose2D newPose) {
        follower.setPose(pose2DtoPose(newPose));
    }

    public Pose2D getPose() {
        return posetoPose2D(follower.getPose());
    }

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = pose2DtoPose(targetPose);
    }

    public double getDistance() {
        return inchToMeter(targetPose.distanceFrom(follower.getPose()));
    }

    public double getVelocityX() {
        return inchToMeter(follower.getVelocity().getXComponent());
    }

    public double getVelocityY() {
        return inchToMeter(follower.getVelocity().getXComponent());
    }

    public double getAngularVelocity() {
        return Math.toDegrees(follower.getAngularVelocity());
    }

    protected Pose pose2DtoPose(@NonNull Pose2D pose2D) {
        Pose pose = new Pose (pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
        return pose;
    }

    protected Pose2D posetoPose2D(@NonNull Pose pose) {
        Pose2D pose2D = new Pose2D (DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
        return pose2D;
    }

    protected double inchToMeter(double inches) {
        return inches*25.4/1000;
    }

    protected double meterToInch(double meters) {
        return meters/25.4*1000;
    }

}
