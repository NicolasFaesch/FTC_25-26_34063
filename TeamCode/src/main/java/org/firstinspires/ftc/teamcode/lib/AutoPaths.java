package org.firstinspires.ftc.teamcode.lib;

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


    // Start Far to other Poses
    public static Path startFarToShootFar;
    public static Path startFarToParkedFar;
    public static Path startFarToGateIntaking;
    public static Path startFarToGateReleasing;


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


    public static Path shootFarToParkedFar;
    public static Path gateReleasingToParkedFar;


    public void buildPaths() {
        // Start Close to other Poses
        startCloseToIntakingClose = createBezierLine(AutoPoses.startPoseClose, AutoPoses.intakingCloseBefore);
        /*
        startCloseToIntakingMiddle;
        startCloseToIntakingFar;
        startCloseToShootClose;
        startCloseToShootCloseParked;
        startCloseToGateIntaking;
        startCloseToGateReleasing;


        // Start Far to other Poses
        startFarToShootFar;
        startFarToParkedFar;
        startFarToGateIntaking;
        startFarToGateReleasing;


        // Shoot Close to Intaking
        shootCloseToIntakingMiddle;
        shootCloseToIntakingFar;
        shootCloseToGateIntaking;
        shootCloseToIntakingClose;
        shootCloseToGateReleasing;
        shootCloseToLoadingZone;


        // Shoot Far to Intaking
        shootFarToIntakingClose;
        shootFarToIntakingMiddle;
        shootFarToIntakingFar;
        shootFarToGateIntaking;
        shootFarToGateReleasing;
        shootFarToLoadingZone;

        // Intaking Paths
        intakingClose;
        intakingMiddle;
        intakingFar;
        intakingLoadingZone;


        // Intaking to Gate Releasing
        intakingCloseToGateReleasing;
        intakingMiddleToGateReleasing;
        intakingFarToGateReleasing;
        loadingZoneToGateReleasing;

        // Objective To Shooting Close
        intakingCloseToShootClose;
        intakingMiddleToShootClose;
        intakingFarToShootClose;
        gateReleasingToShootClose;
        gateIntakingToShootClose;
        loadingZoneToShootClose;

        // Objective To Shooting Far
        intakingCloseToShootFar;
        intakingMiddleToShootFar;
        intakingFarToShootFar;
        gateReleasingToShootFar;
        gateIntakingToShootFar;
        loadingZoneToShootFar;

        // Objective To Shooting Close Parked
        intakingCloseToShootCloseParked;
        intakingMiddleToShootCloseParked;
        intakingFarToShootCloseParked;
        gateReleasingToShootCloseParked;
        gateIntakingToShootCloseParked;
        loadingZoneToShootCloseParked;


        shootFarToParkedFar;
        gateReleasingToParkedFar;

         */
    }



    private Path createBezierLine(AutoPoses.AutoPose startAutoPose, AutoPoses.AutoPose endAutoPose) {
        Pose startPose = startAutoPose.getPose();
        Pose endPose = endAutoPose.getPose();
        Path path = new Path(new BezierLine(startPose, endPose));
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        return path;
    }

}
