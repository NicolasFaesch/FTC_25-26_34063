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
            if (alliance == Alliance.RED) return new Pose(x, y, Math.toRadians(h));
            else return new Pose(x, -y, -Math.toRadians(h));
        }
    }




    // RED Alliance

    // Start
    public static AutoPose startPoseClose = new AutoPose(-49, 48, 129);
    public static AutoPose startPoseFar   = new AutoPose(68, 19, -180);

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

    public static AutoPose shootFar   = new AutoPose(59.3, 14.2, 157.4);

    // Optional (nicht in alten Config aktiv genutzt)
    public static AutoPose shootClosePark = new AutoPose(-45, 19.2, 115.4);


    public static AutoPose parkClose      = new AutoPose(-30, 20, 90); // TODO: pose setzten naja
    public static AutoPose parkFar        = new AutoPose(40, 32.5, 90);

    // Gate (aus gate[0])
    public static AutoPose gateReleasing = new AutoPose( 0, 54, 90);
    public static AutoPose gateReleasingControlPoint = new AutoPose(0, 20, 90);

    // Offen lassen wie gewünscht
    public static AutoPose gateIntaking     = new AutoPose(11, 58, 124);
    public static AutoPose gateIntakingControlPoint = new AutoPose(0, 20, 90);
    public static AutoPose gateIntakingToShooterCloseControlPoint = new AutoPose(0, 20, 90);

    /*
    // Mit 4 Wegen
    public static AutoPose loadingZoneBefore = new AutoPose(64, 26, 80); // TODO: pose setzten naja
    public static AutoPose loadingZoneAdditional1 = new AutoPose(62, 57, 80); // TODO: pose setzten naja
    public static AutoPose loadingZoneAdditional2 = new AutoPose(52, 31, 90); // TODO: pose setzten naja
    public static AutoPose loadingZoneAfter  = new AutoPose(53, 57, 90); // TODO: pose setzten naja
*/

    public static AutoPose loadingZoneBefore = new AutoPose(64, 26, 80); // TODO: pose setzten naja
    public static AutoPose loadingZoneAfter  = new AutoPose(62, 57, 80); // TODO: pose setzten naja


}

