package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class AxonEncoder {
    private AnalogInput encoder;
    public static double offset = 155;
    public static double filterGain = 0.2;

    private double previousAngle;
    private static final double MAX_VOLTAGE = 3.2;

    public AxonEncoder(HardwareMap hwMap, String name) {
        encoder = hwMap.get(AnalogInput.class, name);
        previousAngle = getAngle();
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    public double getRawDegrees() {
        // Divide by the actual max voltage, not the theoretical 3.3
        // Cap the voltage at MAX_VOLTAGE to prevent returning > 360 if there is noise
        double clampedVoltage = Math.min(encoder.getVoltage(), MAX_VOLTAGE);
        return (clampedVoltage / MAX_VOLTAGE) * 360.0;
    }

    public void setZero() {
        offset = getRawDegrees();
    }

    public double getAngle() {
        double currentDegrees = getRawDegrees() - offset;

        // Normalize current reading to 0-360
        if (currentDegrees < 0) currentDegrees += 360.0;
        currentDegrees %= 360.0;

        // 1. Calculate the shortest distance between current and previous
        double diff = currentDegrees - previousAngle;

        // 2. Wrap the difference to be strictly between -180 and +180
        if (diff > 180.0) {
            diff -= 360.0;
        } else if (diff < -180.0) {
            diff += 360.0;
        }

        // 3. Apply the filter ONLY to the difference, and add it to the previous angle
        double angle = previousAngle + (diff * filterGain);

        // 4. Normalize the final result back to 0-360
        if (angle < 0) angle += 360.0;
        angle %= 360.0;

        previousAngle = angle;
        return angle;
    }
}