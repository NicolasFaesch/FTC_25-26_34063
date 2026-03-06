package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.util.Timing;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.pedropathing.paths.Path;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class DrivetrainAuto extends Drivetrain {

    private final double movementTol = 0.7; //in INCH^2
    private Pose2D lastPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private int static_count = 0;
    //CHANGE BACK TO PRIVATE
    public Timing.Timer timer = new Timing.Timer(9999999, TimeUnit.MILLISECONDS);
    private boolean isStoped = false;

    public DrivetrainAuto(HardwareMap hardwareMap, Pose2D startPose) {
        super(hardwareMap, startPose);
        timer.start();
    }

    public void update() {
        follower.update();
        //500 in MS
        if(timer.elapsedTime() >= 500) {
            Pose2D currentPose = this.getPose();
            //relies on early termination to protect from null values
            if (lastPose != null && Math.pow((currentPose.getX(DistanceUnit.INCH) - lastPose.getX(DistanceUnit.INCH)), 2)
                    + Math.pow((currentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH)),2)
                    < movementTol) {
                isStoped = true;
            } else {
                isStoped = false;
            }
            lastPose = currentPose;
            timer.start();
        }
    }

    public boolean isFollowerBusy() {
        return follower.isBusy();
    }

    public boolean isAtEnd() {
        return follower.atParametricEnd();
    }

    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void setFollowerMaxPower(double maxPower) {
        follower.setMaxPower(maxPower);
    }

    public void followPathAndHold(Path path) {
        follower.followPath(path, true);
    }

    public boolean immobile() {
        return isStoped;
    }

    public void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    public void followPathChainAndHold(PathChain pathChain) {
        follower.followPath(pathChain, true);
    }
}
