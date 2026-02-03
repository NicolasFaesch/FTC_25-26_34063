package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseStorage {
    public static final double TARGET_X = -1.83;
    public static final double TARGET_Y = 1.83; // positive on red side
    public static Pose2D currentPose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
    public static final Pose2D targetPoseRed = new Pose2D(DistanceUnit.METER, TARGET_X, TARGET_Y, AngleUnit.DEGREES, 0);
    public static final Pose2D targetPoseBlue = new Pose2D(DistanceUnit.METER, TARGET_X, -TARGET_Y, AngleUnit.DEGREES, 0);
}
