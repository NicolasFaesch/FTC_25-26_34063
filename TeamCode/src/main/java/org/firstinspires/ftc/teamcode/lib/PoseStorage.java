package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseStorage {
    public static final double TARGET_X = -1.83; //TODO: tune
    public static final double TARGET_Y = 1.83; // positive on red side
    public static final double RESET_X = 60;
    public static final double RESET_Y = -62;
    public static final double RESET_HEADING = 180;
    public static Pose2D currentPose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
    public static final Pose2D targetPoseRed = new Pose2D(DistanceUnit.METER, TARGET_X, TARGET_Y, AngleUnit.DEGREES, 0);
    public static final Pose2D targetPoseBlue = new Pose2D(DistanceUnit.INCH, TARGET_X, -TARGET_Y, AngleUnit.DEGREES, 0);
    public static final Pose2D resetPoseRed = new Pose2D(DistanceUnit.INCH, RESET_X, RESET_Y, AngleUnit.DEGREES, RESET_HEADING);
    public static final Pose2D resetPoseBlue = new Pose2D(DistanceUnit.INCH, RESET_X, -RESET_Y, AngleUnit.DEGREES, RESET_HEADING);

    public static final Pose2D parkingPoseRED = new Pose2D(DistanceUnit.METER, (40*2.54)/100,(-32.5*2.54)/100,AngleUnit.DEGREES,0); //Todo
    public static final Pose2D parkingPoseBLUE = new Pose2D(DistanceUnit.METER,(40*2.54)/100,(32.5*2.54)/100,AngleUnit.DEGREES,0); //Todo
    public static double parkingCorrectionX = 0;
    public static double parkingCorrectionY = 0;
    public static final double parkingHeadingRED = 90; //in Degrees.

    public static final double parkingHeadingBLUE = -90; //in Degrees.
}
