package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoConfig {

    public static boolean useWideStart = false;
    public static int maxPickups = 1;
    public static int[] pickupOrder = {0, 1, 2};
    public static double gateWaitTime = 1;


    public static StartConfig[] start = new StartConfig[]{
            new StartConfig(-70, 11, 90),
            new StartConfig(65, 7, 90)
    };


    public static PickupConfig[] pickups = new PickupConfig[]{
            new PickupConfig(-10,   14, -10,    47, 90,
                    true,   false,  0),
            new PickupConfig(13,     20, 13,     60,  90,
                    true,  false,   0),
            new PickupConfig(37,    20, 37,     60, 90,
                    true,   false,  0)
    };

    public static GateConfig[] gate = new GateConfig[]{
            new GateConfig(5, 52, 90)
    };


    public static ShootConfig[] shoot = new ShootConfig[]{
            new ShootConfig(-26, 32, 136), //-15, 18, 143
            new ShootConfig(55, 14, 160)
    };


   /* public static boolean doPark = false;
    public static double parkX = -60, parkY = 60, parkH = 0;*/




    public static class StartConfig {
        public double x, y, h;
        public StartConfig(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    public static class PickupConfig {
        public double xBefore, yBefore, xAfter, yAfter, h;
        public boolean doIntake;   // Vorwärtsbewegung (Y) ausführen
        public boolean doGate;     // Gate-Pose anfahren
        public int shooterPos;     // 0 = Close, 1 = Wide

        public PickupConfig(double xBefore, double yBefore, double xAfter, double yAfter, double h, boolean intake, boolean gate, int sPos) {
            this.xBefore = xBefore; this.yBefore = yBefore; this.xAfter = xAfter; this.yAfter = yAfter; this.h = h;
            this.doIntake = intake; this.doGate = gate; this.shooterPos = sPos;
        }
    }

    public static class GateConfig {
        public double x, y, h;
        public GateConfig(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    public static class ShootConfig {
        public double x, y, h;
        public ShootConfig(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }


}