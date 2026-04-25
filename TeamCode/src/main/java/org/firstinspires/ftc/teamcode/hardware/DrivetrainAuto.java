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



    public DrivetrainAuto(HardwareMap hardwareMap, Pose2D startPose) {
        super(hardwareMap, startPose);
    }

    public void update() {
        follower.update();
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

    public void followPathChain(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    public void followPathChainAndHold(PathChain pathChain) {
        follower.followPath(pathChain, true);
    }
}
