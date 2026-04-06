package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class DynamicAiming {
    /**
     * Handles all dynamic calculations for an FTC shoot-on-the-move turret.
     * Assumes a standard coordinate system where X is forward/backward and Y is left/right.
     */

    // --- Tuning Constants ---

    // Low-Pass Filter Alpha (0.0 to 1.0). Lower = smoother but more lag.
    public static double kVelFilter = 1.0;

    // System lag in seconds (Turret servo settling time + barrel time)
    public static double tLag = 0.12;

    // Lateral drag multiplier (1.0 = vacuum, 0.7 = 30% drag reduction)
    public static double kDrag = 1.2;

    // --- State Variables ---

    // Stored previous loop velocities for the EMA filter
    private static double filteredVx = 0.0;
    private static double filteredVy = 0.0;
    private static double filteredVOmega = 0.0; // Angular velocity (radians/sec)

    private static double targetX = 0.0;
    private static double targetY = 0.0;

    // --- LUT Interpolators ---
    private static InterpLUT hoodLUT;
    private static InterpLUT velocityLUT;
    private static InterpLUT tofLUT;

    public static void createLUTs() {
        hoodLUT = ShooterLUT.createHoodLUT();
        velocityLUT = ShooterLUT.createVelocityLUT();
        tofLUT = ShooterLUT.createTOFLUT();
    }

    public static void setTargetPose(Pose2D targetPose) {
        targetX = targetPose.getX(DistanceUnit.METER);
        targetY = targetPose.getY(DistanceUnit.METER);
    }

    /**
     * Calculates the required turret angle, hood angle, and flywheel RPM.
     * @param robotX       Current robot X position (meters or inches)
     * @param robotY       Current robot Y position
     * @param robotHeading Current robot heading (radians)
     * @param rawVx        Raw X velocity from Odometry
     * @param rawVy        Raw Y velocity from Odometry
     * @param rawVOmega    Raw Angular velocity from Odometry (rad/s)
     * @return A ShooterState object containing the hardware commands
     */
    public static ShooterState calculateTargeting(
            double robotX, double robotY, double robotHeading,
            double rawVx, double rawVy, double rawVOmega) {

        // Filter the Odometry Velocities (Low-Pass EMA Filter)
        filteredVx = (kVelFilter * rawVx) + ((1.0 - kVelFilter) * filteredVx);
        filteredVy = (kVelFilter * rawVy) + ((1.0 - kVelFilter) * filteredVy);
        filteredVOmega = (kVelFilter * rawVOmega) + ((1.0 - kVelFilter) * filteredVOmega);

        // Calculate Initial Distance to get Time of Flight (ToF)
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double initialDistance = Math.hypot(dx, dy);

        // Fetch empirical ToF from LUT
        double tof = tofLUT.get(initialDistance);

        // Calculate Virtual Target (compensating for velocity and drag)
        double virtualTargetX = targetX - (filteredVx * tof * kDrag);
        double virtualTargetY = targetY - (filteredVy * tof * kDrag);

        // Calculate Distance to VIRTUAL Target
        double vtDx = virtualTargetX - robotX;
        double vtDy = virtualTargetY - robotY;
        double virtualDistance = Math.hypot(vtDx, vtDy);

        // Fetch required hood angle and flywheel speed from LUT based on the VIRTUAL distance
        double commandedHoodPos = hoodLUT.get(virtualDistance);
        double commandedRPM = velocityLUT.get(virtualDistance);

        // Calculate Heading to Virtual Target & Apply Lag Feedforward
        // Angle from the robot's current position to the virtual target (field-centric)
        double fieldHeadingToVt = Math.atan2(vtDy, vtDx);

        // Predict where the robot's nose will be pointing after tLag seconds
        double futureRobotHeading = robotHeading + (filteredVOmega * tLag);

        // Final commanded turret angle (Robot-centric)
        double commandedTurretAngle = wrapAngle(fieldHeadingToVt - futureRobotHeading);

        // TODO: replace angle with commandedTurretAngle
        return new ShooterState(Math.atan2(vtDy, vtDx), commandedHoodPos, commandedRPM);
    }

    /**
     * Helper method to keep angles bound between -PI and PI.
     */
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // --- Helper Classes ---

    /**
     * Simple data container for returning the hardware targets.
     */
    public static class ShooterState {
        public double turretAngle; // Radians relative to robot forward
        public double hoodAngle;   // Servo units or degrees
        public double flywheelRpm; // Locked target RPM

        public ShooterState(double turretAngle, double hoodAngle, double flywheelRpm) {
            this.turretAngle = turretAngle;
            this.hoodAngle = hoodAngle;
            this.flywheelRpm = flywheelRpm;
        }
    }

}
