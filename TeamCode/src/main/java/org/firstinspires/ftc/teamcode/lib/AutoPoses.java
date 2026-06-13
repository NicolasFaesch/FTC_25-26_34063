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

    public static AutoPose startPoseClose = new AutoPose(-49, 48, 0);
    public static AutoPose startPoseFar   = new AutoPose(60, 19, 90);

    // Intake BEFORE (xBefore, yBefore, h)
    public static AutoPose intakingCloseBefore  = new AutoPose(-12.5, 25, 90);
    public static AutoPose intakingMiddleBefore = new AutoPose(13, 25, 90);
    public static AutoPose intakingFarBefore    = new AutoPose(37, 25, 90);

    // Intake AFTER (xAfter, yAfter, h)
    public static AutoPose intakingCloseAfter  = new AutoPose(-12.5, 50, 90);
    public static AutoPose intakingMiddleAfter = new AutoPose(12, 50, 90);
    public static AutoPose intakingFarAfter    = new AutoPose(37, 50, 90);

    // Shooting
    public static AutoPose shootClose = new AutoPose(-19, 22.5, 30);

    public static AutoPose shootFar   = new AutoPose(59.3, 14.2, 90);

    // Optional (nicht in alten Config aktiv genutzt)
    public static AutoPose shootClosePark = new AutoPose(-45, 19.2, 115.4);


    public static AutoPose parkClose      = new AutoPose(-45, 20, 90); // TODO: pose setzten naja
    public static AutoPose parkFar        = new AutoPose(60, 54, 90); // 40, 32.5, 90

    // Gate (aus gate[0])
    public static AutoPose gateReleasing = new AutoPose( 5, 52, 90);
    public static AutoPose gateReleasingControlPoint = new AutoPose(5, 20, 90);

    // Offen lassen wie gewünscht
    public static AutoPose gateIntaking     = new AutoPose(10, 54, 130);
    //public static AutoPose gateIntakingTele = new AutoPose(13, 55, 125);
    public static AutoPose gateIntakingTele = new AutoPose(15, 55, 120);
    public static AutoPose gateIntakingSecond = new AutoPose(16.5, 57, 145);
    public static AutoPose gateIntakingControlPoint = new AutoPose(12, 20, 128);
    public static AutoPose gateIntakingToShooterCloseControlPoint = new AutoPose(0, 20, 90);

    /*
    // Mit 4 Wegen
    public static AutoPose loadingZoneBefore = new AutoPose(64, 26, 80); // TODO: pose setzten naja
    public static AutoPose loadingZoneAdditional1 = new AutoPose(62, 57, 80); // TODO: pose setzten naja
    public static AutoPose loadingZoneAdditional2 = new AutoPose(52, 31, 90); // TODO: pose setzten naja
    public static AutoPose loadingZoneAfter  = new AutoPose(53, 57, 90); // TODO: pose setzten naja*/


    public static AutoPose loadingZoneBefore = new AutoPose(64, 45, 80); // TODO: pose setzten naja
    public static AutoPose loadingZoneAfter  = new AutoPose(64, 62, 80); // TODO: pose setzten naja

    public static AutoPose CP1  = new AutoPose(0, 0, 0); // TODO: pose setzten naja
    public static AutoPose CP2  = new AutoPose(0, 0, 0); // TODO: pose setzten naja

    public static AutoPose parkingPoseTele = new AutoPose(38, -37, 180);
    public static AutoPose parkingPoseTeleControlPoint = new AutoPose(parkingPoseTele.x-20, parkingPoseTele.y, parkingPoseTele.h);
}

