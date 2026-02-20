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
            this.h = Math.toRadians(h);
        }

        public Pose getPose() {
            if (alliance == Alliance.RED) return new Pose(x, y, h);
            else return new Pose(x, -y, -h);
        }
    }




    // RED Alliance

    // Start
    public static AutoPose startPoseClose = new AutoPose(-70, 11, 90);
    public static AutoPose startPoseFar   = new AutoPose(65, 7, 90);

    // Intake BEFORE (xBefore, yBefore, h)
    public static AutoPose intakingCloseBefore  = new AutoPose(-14, 27, 90);
    public static AutoPose intakingMiddleBefore = new AutoPose(12, 27, 90);
    public static AutoPose intakingFarBefore    = new AutoPose(35, 27, 90);

    // Intake AFTER (xAfter, yAfter, h)
    public static AutoPose intakingCloseAfter  = new AutoPose(-14, 49, 90);
    public static AutoPose intakingMiddleAfter = new AutoPose(12, 49, 90);
    public static AutoPose intakingFarAfter    = new AutoPose(35, 49, 90);

    // Shooting
    public static AutoPose shootClose = new AutoPose(-19, 22.5, 136);
    public static AutoPose shootFar   = new AutoPose(55, 14, 160);

    // Optional (nicht in alten Config aktiv genutzt)
    public static AutoPose shootClosePark = new AutoPose(-25, 11.5, 130);
    public static AutoPose parkFar        = new AutoPose(0, 0, 0);

    // Gate (aus gate[0])
    public static AutoPose gateReleasing = new AutoPose(0, 54, 90);

    // Offen lassen wie gewünscht
    public static AutoPose gateIntaking     = new AutoPose(0, 0, 0);
    public static AutoPose loadingZoneBefore = new AutoPose(0, 0, 0);
    public static AutoPose loadingZoneAfter  = new AutoPose(0, 0, 0);

    public static AutoPose gateReleasingControlPoint = new AutoPose(0, 20, 90);




}

