package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.ArrayList;
import java.util.List;

public class ShooterLUT {
    private static List<double[]> shooterPositions = new ArrayList<>();

    // known good position points for LUT, goes as DISTANCE, VELOCITY, HOOD POSITION
    static {
        shooterPositions.add(new double[]{0.75, 2500, 0.425});
        shooterPositions.add(new double[]{0.9, 2750, 0.6});
        shooterPositions.add(new double[]{1.2, 2750, 0.6});
        shooterPositions.add(new double[]{1.3, 2750, 0.675});
        shooterPositions.add(new double[]{1.4, 3000, 0.725});
        shooterPositions.add(new double[]{1.5, 3000, 0.75});
        shooterPositions.add(new double[]{1.6, 3000, 0.75});
        shooterPositions.add(new double[]{1.7, 3000, 0.775});
        shooterPositions.add(new double[]{1.8, 3100, 0.775});
        shooterPositions.add(new double[]{1.9, 3150, 0.775});
        shooterPositions.add(new double[]{2.0, 3250, 0.775});
        shooterPositions.add(new double[]{2.1, 3250, 0.8});
        shooterPositions.add(new double[]{2.2, 3250, 0.8});
        shooterPositions.add(new double[]{2.3, 3300, 0.825});
        shooterPositions.add(new double[]{2.4, 3350, 0.85});
        shooterPositions.add(new double[]{2.5, 3500, 0.85});
        shooterPositions.add(new double[]{2.6, 3500, 0.85});
        shooterPositions.add(new double[]{2.7, 3500, 0.85});
        shooterPositions.add(new double[]{3.7, 4000, 0.875});
    }

    private static InterpLUT velocityLUT;
    private static InterpLUT hoodLUT;

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
}
