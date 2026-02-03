package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PositionChecker {
    private static final double ROBOT_RADIUS = 0.175; // smallest distance from robot center to outside
    private static final double TOLERANCE = 0.0; // extra tolerance for check

    private static final Pose2D NEAR_ZONE_CENTER = new Pose2D (DistanceUnit.METER, -1.83, 0, AngleUnit.DEGREES, 0);
    private static final Pose2D FAR_ZONE_CENTER = new Pose2D (DistanceUnit.METER, 1.83, 0, AngleUnit.DEGREES, 0);

    private static final double NEAR_ZONE_WIDTH = 1.83;
    private static final double FAR_ZONE_WIDTH = 0.61;


    private static double l1Norm (double x, double y) {
        return Math.abs(x) + Math.abs(y);
    }

    // zoneWidth describes the x or y distance to the corner from the center
    private static boolean checkInZone(Pose2D robotPose, Pose2D zoneCenterPose, double zoneWidth) {
        double dX = robotPose.getX(DistanceUnit.METER) - zoneCenterPose.getX(DistanceUnit.METER);
        double dY = robotPose.getY(DistanceUnit.METER) - zoneCenterPose.getY(DistanceUnit.METER);
        return (l1Norm(dX, dY)-ROBOT_RADIUS) <= zoneWidth + TOLERANCE;
    }

    public static boolean checkInZones(Pose2D robotPose) {
        return (checkInZone(robotPose, NEAR_ZONE_CENTER, NEAR_ZONE_WIDTH)
                || checkInZone(robotPose, FAR_ZONE_CENTER, FAR_ZONE_WIDTH));
    }
}
