package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseStorage {
    public static Pose2D currentPose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
}
