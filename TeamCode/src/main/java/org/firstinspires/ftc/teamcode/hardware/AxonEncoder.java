package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonEncoder {
    private AnalogInput encoder;
    private double offset = 0;

    public AxonEncoder(HardwareMap hwMap, String name) {
        encoder = hwMap.get(AnalogInput.class, name);
    }

    /**
     * @return The current voltage from the 4th wire (0 to 3.3V)
     */
    public double getVoltage() {
        return encoder.getVoltage();
    }

    /**
     * Axon servos are typically 360-degree sensors.
     * @return Position in degrees (0-360)
     */
    public double getRawDegrees() {
        // Voltage / MaxVoltage * Range
        return (encoder.getVoltage() / 3.3) * 360.0;
    }

    /**
     * Sets the current position as 0.
     */
    public void setZero() {
        offset = getRawDegrees();
    }

    /**
     * @return Degrees relative to your zero point, normalized to 0-360.
     */
    public double getAngle() {
        double degrees = getRawDegrees() - offset;
        if (degrees < 0) degrees += 360;
        return degrees;
    }
}