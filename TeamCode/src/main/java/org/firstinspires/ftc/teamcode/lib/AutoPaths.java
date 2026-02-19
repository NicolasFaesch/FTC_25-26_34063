package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import java.util.List;

public class AutoPaths {

    // Start Close to other Poses
    public static Path startCloseToIntakingClose;
    public static Path startCloseToIntakingMiddle;
    public static Path startCloseToIntakingFar;
    public static Path startCloseToShootClose;
    public static Path startCloseToShootCloseParked;
    public static Path startCloseToGateIntaking;
    public static Path startCloseToGateReleasing;
    public static Path startCloseToLoadingZone;


    // Start Far to other Poses
    public static Path startFarToIntakingClose;
    public static Path startFarToIntakingMiddle;
    public static Path startFarToIntakingFar;
    public static Path startFarToShootFar;
    public static Path startFarToParkedFar;
    public static Path startFarToGateIntaking;
    public static Path startFarToGateReleasing;
    public static Path startFarToLoadingZone;


    // Shoot Close to Intaking
    public static Path shootCloseToIntakingMiddle;
    public static Path shootCloseToIntakingFar;
    public static Path shootCloseToGateIntaking;
    public static Path shootCloseToIntakingClose;
    public static Path shootCloseToGateReleasing;
    public static Path shootCloseToLoadingZone;


    // Shoot Far to Intaking
    public static Path shootFarToIntakingClose;
    public static Path shootFarToIntakingMiddle;
    public static Path shootFarToIntakingFar;
    public static Path shootFarToGateIntaking;
    public static Path shootFarToGateReleasing;
    public static Path shootFarToLoadingZone;

    // Intaking Paths
    public static Path intakingClose;
    public static Path intakingMiddle;
    public static Path intakingFar;
    public static Path intakingLoadingZone;


    // Intaking to Gate Releasing
    public static Path intakingCloseToGateReleasing;
    public static Path intakingMiddleToGateReleasing;
    public static Path intakingFarToGateReleasing;
    public static Path loadingZoneToGateReleasing;

    // Gate Releasing to Intaking
    public static Path gateReleasingToIntakingClose;
    public static Path gateReleasingToIntakingMiddle;
    public static Path gateReleasingToIntakingFar;
    public static Path gateReleasingToLoadingZone;

    // Objective To Shooting Close
    public static Path intakingCloseToShootClose;
    public static Path intakingMiddleToShootClose;
    public static Path intakingFarToShootClose;
    public static Path gateReleasingToShootClose;
    public static Path gateIntakingToShootClose;
    public static Path loadingZoneToShootClose;

    // Objective To Shooting Far
    public static Path intakingCloseToShootFar;
    public static Path intakingMiddleToShootFar;
    public static Path intakingFarToShootFar;
    public static Path gateReleasingToShootFar;
    public static Path gateIntakingToShootFar;
    public static Path loadingZoneToShootFar;

    // Objective To Shooting Close Parked
    public static Path intakingCloseToShootCloseParked;
    public static Path intakingMiddleToShootCloseParked;
    public static Path intakingFarToShootCloseParked;
    public static Path gateReleasingToShootCloseParked;
    public static Path gateIntakingToShootCloseParked;
    public static Path loadingZoneToShootCloseParked;
    public static Path shootCloseToShootCloseParked;


    public static Path shootFarToParkedFar;
    public static Path gateReleasingToParkedFar;


    public void buildPaths() {
        // Start Close to other Poses
        startCloseToIntakingClose = createBezierLine(AutoPoses.startPoseClose, AutoPoses.intakingCloseBefore);
        startCloseToIntakingMiddle = createBezierLine(AutoPoses.startPoseClose, AutoPoses.intakingMiddleBefore);
        startCloseToIntakingFar = createBezierLine(AutoPoses.startPoseClose, AutoPoses.intakingFarBefore);
        startCloseToShootClose = createBezierLine(AutoPoses.startPoseClose, AutoPoses.shootClose);
        startCloseToShootCloseParked = createBezierLine(AutoPoses.startPoseClose, AutoPoses.shootClosePark);
        startCloseToGateIntaking = createBezierLine(AutoPoses.startPoseClose, AutoPoses.gateIntaking);
        startCloseToGateReleasing = createBezierLine(AutoPoses.startPoseClose, AutoPoses.gateReleasing);
        startCloseToLoadingZone = createBezierLine(AutoPoses.startPoseClose, AutoPoses.loadingZoneBefore);

        // Start Far to other Poses
        startFarToIntakingClose = createBezierLine(AutoPoses.startPoseFar, AutoPoses.intakingCloseBefore);
        startFarToIntakingMiddle = createBezierLine(AutoPoses.startPoseFar, AutoPoses.intakingMiddleBefore);
        startFarToIntakingFar = createBezierLine(AutoPoses.startPoseFar, AutoPoses.intakingFarBefore);
        startFarToShootFar = createBezierLine(AutoPoses.startPoseFar, AutoPoses.shootFar);
        startFarToParkedFar = createBezierLine(AutoPoses.startPoseFar, AutoPoses.parkFar);
        startFarToGateIntaking = createBezierLine(AutoPoses.startPoseFar, AutoPoses.gateIntaking);
        startFarToGateReleasing = createBezierLine(AutoPoses.startPoseFar, AutoPoses.gateReleasing);
        startFarToLoadingZone = createBezierLine(AutoPoses.startPoseFar, AutoPoses.loadingZoneBefore);

        // Shoot Close to Intaking
        shootCloseToIntakingClose = createBezierLine(AutoPoses.shootClose, AutoPoses.intakingCloseBefore);
        shootCloseToIntakingMiddle = createBezierLine(AutoPoses.shootClose, AutoPoses.intakingMiddleBefore);
        shootCloseToIntakingFar = createBezierLine(AutoPoses.shootClose, AutoPoses.intakingFarBefore);
        shootCloseToGateIntaking = createBezierLine(AutoPoses.shootClose, AutoPoses.gateIntaking);
        shootCloseToLoadingZone = createBezierLine(AutoPoses.shootClose, AutoPoses.loadingZoneBefore);

        // Shoot Far to Intaking
        shootFarToIntakingClose = createBezierLine(AutoPoses.shootFar, AutoPoses.intakingCloseBefore);
        shootFarToIntakingMiddle = createBezierLine(AutoPoses.shootFar, AutoPoses.intakingMiddleBefore);
        shootFarToIntakingFar = createBezierLine(AutoPoses.shootFar, AutoPoses.intakingFarBefore);
        shootFarToGateIntaking = createBezierLine(AutoPoses.shootFar, AutoPoses.gateIntaking);
        shootFarToLoadingZone = createBezierLine(AutoPoses.shootFar, AutoPoses.loadingZoneBefore);

        // To Gate Releasing
        shootCloseToGateReleasing = createBezierCurve(AutoPoses.shootClose, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        shootFarToGateReleasing = createBezierCurve(AutoPoses.shootFar, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        intakingCloseToGateReleasing = createBezierCurve(AutoPoses.intakingCloseAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        intakingMiddleToGateReleasing = createBezierCurve(AutoPoses.intakingMiddleAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        intakingFarToGateReleasing = createBezierCurve(AutoPoses.intakingFarAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        loadingZoneToGateReleasing = createBezierCurve(AutoPoses.loadingZoneAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);


        // Intaking Paths
        intakingClose = createBezierLine(AutoPoses.intakingCloseBefore, AutoPoses.intakingCloseAfter);
        intakingMiddle = createBezierLine(AutoPoses.intakingMiddleBefore, AutoPoses.intakingMiddleAfter);
        intakingFar = createBezierLine(AutoPoses.intakingFarBefore, AutoPoses.intakingFarAfter);
        intakingLoadingZone = createBezierLine(AutoPoses.loadingZoneBefore, AutoPoses.loadingZoneAfter);

        // Gate Releasing to Intaking
        gateReleasingToIntakingClose = createBezierLine(AutoPoses.gateReleasing, AutoPoses.intakingCloseAfter);
        gateReleasingToIntakingMiddle = createBezierLine(AutoPoses.gateReleasing, AutoPoses.intakingMiddleAfter);
        gateReleasingToIntakingFar = createBezierLine(AutoPoses.gateReleasing, AutoPoses.intakingFarAfter);
        gateReleasingToLoadingZone = createBezierLine(AutoPoses.gateReleasing, AutoPoses.loadingZoneAfter);

        // Objective To Shooting Close
        intakingCloseToShootClose = createBezierLine(AutoPoses.intakingCloseAfter, AutoPoses.shootClose);
        intakingMiddleToShootClose = createBezierLine(AutoPoses.intakingMiddleAfter, AutoPoses.shootClose);
        intakingFarToShootClose = createBezierLine(AutoPoses.intakingFarAfter, AutoPoses.shootClose);
        gateReleasingToShootClose = createBezierLine(AutoPoses.gateReleasing, AutoPoses.shootClose);
        gateIntakingToShootClose = createBezierLine(AutoPoses.gateIntaking, AutoPoses.shootClose);
        loadingZoneToShootClose = createBezierLine(AutoPoses.loadingZoneAfter, AutoPoses.shootClose);

        // Objective To Shooting Far
        intakingCloseToShootFar = createBezierLine(AutoPoses.intakingCloseAfter, AutoPoses.shootFar);
        intakingMiddleToShootFar = createBezierLine(AutoPoses.intakingMiddleAfter, AutoPoses.shootFar);
        intakingFarToShootFar = createBezierLine(AutoPoses.intakingFarAfter, AutoPoses.shootFar);
        gateReleasingToShootFar = createBezierLine(AutoPoses.gateReleasing, AutoPoses.shootFar);
        gateIntakingToShootFar = createBezierLine(AutoPoses.gateIntaking, AutoPoses.shootFar);
        loadingZoneToShootFar = createBezierLine(AutoPoses.loadingZoneAfter, AutoPoses.shootFar);

        // Objective To Shooting Close Parked
        intakingCloseToShootCloseParked = createBezierLine(AutoPoses.intakingCloseAfter, AutoPoses.shootClosePark);
        intakingMiddleToShootCloseParked = createBezierLine(AutoPoses.intakingMiddleAfter, AutoPoses.shootClosePark);
        intakingFarToShootCloseParked = createBezierLine(AutoPoses.intakingFarAfter, AutoPoses.shootClosePark);
        gateReleasingToShootCloseParked = createBezierLine(AutoPoses.gateReleasing, AutoPoses.shootClosePark);
        gateIntakingToShootCloseParked = createBezierLine(AutoPoses.gateIntaking, AutoPoses.shootClosePark);
        loadingZoneToShootCloseParked = createBezierLine(AutoPoses.loadingZoneAfter, AutoPoses.shootClosePark);
        shootCloseToShootCloseParked = createBezierLine(AutoPoses.startPoseClose, AutoPoses.shootClosePark);

        // Remaining Paths
        shootFarToParkedFar = createBezierLine(AutoPoses.shootFar, AutoPoses.parkFar);
        gateReleasingToParkedFar = createBezierLine(AutoPoses.gateReleasing, AutoPoses.parkFar);
    }




    private Path createBezierLine(AutoPoses.AutoPose startAutoPose, AutoPoses.AutoPose endAutoPose) {
        Pose startPose = startAutoPose.getPose();
        Pose endPose = endAutoPose.getPose();
        Path path = new Path(new BezierLine(startPose, endPose));
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        return path;
    }

    private Path createBezierCurve(AutoPoses.AutoPose startAutoPose, AutoPoses.AutoPose controlPoint, AutoPoses.AutoPose endAutoPose) {
        Pose startPose = startAutoPose.getPose();
        Pose controlPose = controlPoint.getPose();
        Pose endPose = endAutoPose.getPose();
        Path path = new Path(new BezierCurve(startPose, controlPose, endPose));
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        return path;
    }

}
