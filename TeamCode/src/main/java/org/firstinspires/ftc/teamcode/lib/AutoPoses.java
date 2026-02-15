package org.firstinspires.ftc.teamcode.lib;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import com.pedropathing.geometry.Pose;

@Configurable
public class AutoPoses {
    public enum Alliance {
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.RED;

    public static class AutoPose {
        @Sorter(sort = 0)
        public final double x;
        @Sorter(sort = 1)
        public final double y;
        @Sorter(sort = 2)
        public final double h;

        public AutoPose(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public Pose getPose() {
            if (alliance == Alliance.RED) return new Pose(x, y, h);
            else return new Pose(x, -y, -h);
        }
    }




    // Original Poses all for RED alliance

    public static AutoPose startPoseClose = new AutoPose(0, 0, 0);

    public static AutoPose startPoseFar = new AutoPose(0, 0, 0);

    public static AutoPose intakingCloseBefore = new AutoPose(0, 0, 0);
    public static AutoPose intakingMiddleBefore = new AutoPose(0, 0, 0);

    public static AutoPose intakingFarBefore = new AutoPose(0, 0, 0);

    public static AutoPose intakingCloseAfter = new AutoPose(0, 0, 0);

    public static AutoPose intakingMiddleAfter = new AutoPose(0, 0, 0);

    public static AutoPose intakingFarAfter = new AutoPose(0, 0, 0);

    public static AutoPose shootClose = new AutoPose(0, 0, 0);

    public static AutoPose shootFar = new AutoPose(0, 0, 0);

    public static AutoPose shootClosePark = new AutoPose(0, 0, 0);

    public static AutoPose parkFar = new AutoPose(0, 0, 0);

    public static AutoPose gateIntaking = new AutoPose(0, 0, 0);

    public static AutoPose gateReleasing = new AutoPose(0, 0, 0);

    public static AutoPose loadingZoneBefore = new AutoPose(0, 0, 0);

    public static AutoPose loadingZoneAfter = new AutoPose(0, 0, 0);



}

