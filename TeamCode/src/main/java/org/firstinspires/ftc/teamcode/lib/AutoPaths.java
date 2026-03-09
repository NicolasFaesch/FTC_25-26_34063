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

    // parked Close poses
    public static Path startCloseToParkedClose;
    public static Path shootCloseToParkedClose;
    public static Path gateReleasingToParkedClose;


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

    public static List<Path> loadingZoneChain;

    public static Path intakingLoadingZone1;
    public static Path intakingLoadingZone2;
    public static Path intakingLoadingZone3;


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
        startCloseToIntakingMiddle = createBezierCurve(AutoPoses.startPoseClose, AutoPoses.CP1, AutoPoses.intakingMiddleBefore);
        startCloseToIntakingFar = createBezierLine(AutoPoses.startPoseClose, AutoPoses.intakingFarBefore);
        startCloseToShootClose = createBezierLine(AutoPoses.startPoseClose, AutoPoses.shootClose);
        startCloseToShootCloseParked = createBezierLine(AutoPoses.startPoseClose, AutoPoses.shootClosePark);
        startCloseToGateIntaking = createBezierCurve(AutoPoses.startPoseClose, AutoPoses.gateIntakingControlPoint, AutoPoses.gateIntaking);

        // Set Path Braking Start
        startCloseToGateIntaking.setBrakingStrength(AutoConstants.GATE_INTAKING_BREAK_STRENGTH);

        startCloseToGateReleasing = createBezierLine(AutoPoses.startPoseClose, AutoPoses.gateReleasing);
        startCloseToLoadingZone = createBezierLine(AutoPoses.startPoseClose, AutoPoses.loadingZoneBefore);

        // Start Far to other Poses
        startFarToIntakingClose = createBezierLine(AutoPoses.startPoseFar, AutoPoses.intakingCloseBefore);
        startFarToIntakingMiddle = createBezierLine(AutoPoses.startPoseFar, AutoPoses.intakingMiddleBefore);
        startFarToIntakingFar = createBezierLine(AutoPoses.startPoseFar, AutoPoses.intakingFarBefore);
        startFarToShootFar = createBezierLine(AutoPoses.startPoseFar, AutoPoses.shootFar);
        startFarToParkedFar = createBezierLine(AutoPoses.startPoseFar, AutoPoses.parkFar);
        startFarToGateIntaking = createBezierCurve(AutoPoses.startPoseFar, AutoPoses.gateIntakingControlPoint, AutoPoses.gateIntaking);

        // Set Path Braking Start
        startFarToGateIntaking.setBrakingStrength(AutoConstants.GATE_INTAKING_BREAK_STRENGTH);

        startFarToGateReleasing = createBezierLine(AutoPoses.startPoseFar, AutoPoses.gateReleasing);
        startFarToLoadingZone = createBezierLine(AutoPoses.startPoseFar, AutoPoses.loadingZoneBefore);

        // Shoot Close to Intaking
        shootCloseToIntakingClose = createBezierLine(AutoPoses.shootClose, AutoPoses.intakingCloseBefore);
        shootCloseToIntakingMiddle = createBezierCurve(AutoPoses.shootClose, AutoPoses.CP1, AutoPoses.intakingMiddleBefore);
        shootCloseToIntakingFar = createBezierLine(AutoPoses.shootClose, AutoPoses.intakingFarBefore);
        shootCloseToGateIntaking = createBezierCurve(AutoPoses.shootClose, AutoPoses.gateIntakingControlPoint, AutoPoses.gateIntaking);

        // Set Path Braking Start
        shootCloseToGateIntaking.setBrakingStrength(AutoConstants.GATE_INTAKING_BREAK_STRENGTH);

        shootCloseToLoadingZone = createBezierLine(AutoPoses.shootClose, AutoPoses.loadingZoneBefore);

        // Shoot Far to Intaking
        shootFarToIntakingClose = createBezierLine(AutoPoses.shootFar, AutoPoses.intakingCloseBefore);
        shootFarToIntakingMiddle = createBezierLine(AutoPoses.shootFar, AutoPoses.intakingMiddleBefore);
        shootFarToIntakingFar = createBezierLine(AutoPoses.shootFar, AutoPoses.intakingFarBefore);
        shootFarToGateIntaking = createBezierCurve(AutoPoses.shootFar, AutoPoses.gateIntakingControlPoint, AutoPoses.gateIntaking);

        // Set Path Braking Start
        shootFarToGateIntaking.setBrakingStrength(AutoConstants.GATE_INTAKING_BREAK_STRENGTH);

        shootFarToLoadingZone = createBezierLine(AutoPoses.shootFar, AutoPoses.loadingZoneBefore);

        // To Gate Releasing
        shootCloseToGateReleasing = createBezierCurve(AutoPoses.shootClose, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        shootFarToGateReleasing = createBezierCurve(AutoPoses.shootFar, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        intakingCloseToGateReleasing = createBezierCurve(AutoPoses.intakingCloseAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        intakingMiddleToGateReleasing = createBezierCurve(AutoPoses.intakingMiddleAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        intakingFarToGateReleasing = createBezierCurve(AutoPoses.intakingFarAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);
        loadingZoneToGateReleasing = createBezierCurve(AutoPoses.loadingZoneAfter, AutoPoses.gateReleasingControlPoint, AutoPoses.gateReleasing);

        // Set Path Braking Start
        shootCloseToGateReleasing.setBrakingStrength(AutoConstants.GATE_BREAK_STRENGTH);
        shootFarToGateReleasing.setBrakingStrength(AutoConstants.GATE_BREAK_STRENGTH);
        intakingMiddleToGateReleasing.setBrakingStrength(AutoConstants.GATE_BREAK_STRENGTH);
        intakingFarToGateReleasing.setBrakingStrength(AutoConstants.GATE_BREAK_STRENGTH);
        loadingZoneToGateReleasing.setBrakingStrength(AutoConstants.GATE_BREAK_STRENGTH);

        // Intaking Paths
        intakingClose = createBezierLine(AutoPoses.intakingCloseBefore, AutoPoses.intakingCloseAfter);
        intakingMiddle = createBezierLine(AutoPoses.intakingMiddleBefore, AutoPoses.intakingMiddleAfter);
        intakingFar = createBezierLine(AutoPoses.intakingFarBefore, AutoPoses.intakingFarAfter);

    /*    intakingLoadingZone1 = createBezierLine(AutoPoses.loadingZoneBefore, AutoPoses.loadingZoneAdditional1);
        intakingLoadingZone2 = createBezierLine(AutoPoses.loadingZoneAdditional1, AutoPoses.loadingZoneAdditional2);
        intakingLoadingZone3 = createBezierLine(AutoPoses.loadingZoneAdditional2, AutoPoses.loadingZoneAfter);

        loadingZoneChain = List.of( intakingLoadingZone1, intakingLoadingZone2, intakingLoadingZone3 );*/

        intakingLoadingZone1 = createBezierLine(AutoPoses.loadingZoneBefore, AutoPoses.loadingZoneAfter);

        loadingZoneChain = List.of( intakingLoadingZone1);

        // Gate Releasing to Intaking
        gateReleasingToIntakingClose = createBezierCurve(AutoPoses.gateReleasing, AutoPoses.gateReleasingControlPoint, AutoPoses.intakingCloseBefore);
        gateReleasingToIntakingMiddle = createBezierCurve(AutoPoses.gateReleasing, AutoPoses.gateReleasingControlPoint, AutoPoses.intakingMiddleBefore);
        gateReleasingToIntakingFar = createBezierCurve(AutoPoses.gateReleasing, AutoPoses.gateReleasingControlPoint,  AutoPoses.intakingFarBefore);
        gateReleasingToLoadingZone = createBezierCurve(AutoPoses.gateReleasing, AutoPoses.gateReleasingControlPoint, AutoPoses.loadingZoneBefore);

        // Objective To Shooting Close
        intakingCloseToShootClose = createBezierLine(AutoPoses.intakingCloseAfter, AutoPoses.shootClose);
        intakingMiddleToShootClose = createBezierLine(AutoPoses.intakingMiddleAfter, AutoPoses.shootClose);
        intakingFarToShootClose = createBezierLine(AutoPoses.intakingFarAfter, AutoPoses.shootClose);
        gateReleasingToShootClose = createBezierCurve(AutoPoses.gateReleasing, AutoPoses.CP2, AutoPoses.shootClose);
        gateIntakingToShootClose = createBezierCurve(AutoPoses.gateIntaking, AutoPoses.gateIntakingToShooterCloseControlPoint, AutoPoses.shootClose);
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


        startCloseToParkedClose = createBezierLine(AutoPoses.startPoseFar, AutoPoses.parkClose);
        shootCloseToParkedClose = createBezierLine(AutoPoses.shootFar, AutoPoses.parkClose);
        gateReleasingToParkedClose = createBezierLine(AutoPoses.gateReleasing, AutoPoses.parkClose);
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
