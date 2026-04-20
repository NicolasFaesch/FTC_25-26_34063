package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.ArrayList;
import java.util.List;

public class ShooterLUT {
    private static List<double[]> shooterPositions = new ArrayList<>();

    // known good position points for LUT, goes as DISTANCE, VELOCITY, HOOD POSITION, TIME_OF_FLIGHT
    static {
        shooterPositions.add(new double[]{0.9, 2800, 0.175, 0.55});
        shooterPositions.add(new double[]{1.0, 2800, 0.175});
        shooterPositions.add(new double[]{1.1, 2700, 0.225});
        shooterPositions.add(new double[]{1.2, 2700, 0.275});
        shooterPositions.add(new double[]{1.3, 2800, 0.275});
        shooterPositions.add(new double[]{1.4, 2800, 0.325});
        shooterPositions.add(new double[]{1.5, 2900, 0.325, 0.6});
        shooterPositions.add(new double[]{1.6, 3000, 0.35});
        shooterPositions.add(new double[]{1.7, 3100, 0.375});
        shooterPositions.add(new double[]{1.8, 3100, 0.4});
        shooterPositions.add(new double[]{1.9, 3100, 0.4});
        shooterPositions.add(new double[]{2.0, 3200, 0.425, 0.7});
        shooterPositions.add(new double[]{2.1, 3200, 0.425});
        shooterPositions.add(new double[]{2.2, 3300, 0.45});
        shooterPositions.add(new double[]{2.3, 3300, 0.475});
        shooterPositions.add(new double[]{2.4, 3400, 0.5});
        shooterPositions.add(new double[]{2.5, 3500, 0.5, 0.8});
        shooterPositions.add(new double[]{2.6, 3500, 0.5});
        shooterPositions.add(new double[]{2.7, 3600, 0.525});
        shooterPositions.add(new double[]{3.6, 4450, 0.6875});
        shooterPositions.add(new double[]{3.8, 4450, 0.6875});
        shooterPositions.add(new double[]{3.9, 4500, 0.6875, 0.8});
    }

    public static double minDistance = shooterPositions.get(0)[0];

    private static InterpLUT velocityLUT;
    private static InterpLUT hoodLUT;
    private static InterpLUT tofLUT;

    public static InterpLUT createVelocityLUT() {
        velocityLUT = new InterpLUT();
        velocityLUT.add(-10, shooterPositions.get(0)[1]); // add physically unreachable lower value to extend range
        for (int i = 0; i < shooterPositions.size(); i++) {
            velocityLUT.add(shooterPositions.get(i)[0], shooterPositions.get(i)[1]);
        }
        velocityLUT.add(10, shooterPositions.get(shooterPositions.size()-1)[1]); // add physically unreachable higher value to extend range

        velocityLUT.createLUT();
        return velocityLUT;
    }

    public static InterpLUT createHoodLUT() {
        hoodLUT = new InterpLUT();
        hoodLUT.add(-10, shooterPositions.get(0)[2]); // add physically unreachable lower value to extend range
        for (int i = 0; i < shooterPositions.size(); i++) {
            hoodLUT.add(shooterPositions.get(i)[0], shooterPositions.get(i)[2]);
        }
        hoodLUT.add(10, shooterPositions.get(shooterPositions.size()-1)[2]); // add physically unreachable higher value to extend range

        hoodLUT.createLUT();
        return hoodLUT;
    }

    public static InterpLUT createTOFLUT() {
        tofLUT = new InterpLUT();
        tofLUT.add(-10, shooterPositions.get(0)[3]); // add physically unreachable lower value to extend range
        for (int i = 0; i < shooterPositions.size(); i++) {
            if (shooterPositions.get(i).length > 3) {
                tofLUT.add(shooterPositions.get(i)[0], shooterPositions.get(i)[3]);
            }        }
        tofLUT.add(10, shooterPositions.get(shooterPositions.size()-1)[3]); // add physically unreachable higher value to extend range

        tofLUT.createLUT();
        return tofLUT;
    }
}
