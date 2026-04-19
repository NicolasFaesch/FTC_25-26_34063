package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(
        name = "AS5600 Magnetic Encoder",
        xmlTag = "AS5600",
        description = "AS5600 12-bit Absolute Encoder"
)
public class AS5600Driver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

    public static final byte DEFAULT_ADDRESS = 0x36;
    private double offsetDegrees = 0;

    private enum Register {
        STATUS(0x0B),
        RAW_ANGLE_HI(0x0C),
        AGC(0x1A); // Automatic Gain Control register
        private final int bVal;
        Register(int bVal) { this.bVal = bVal; }
    }

    public AS5600Driver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    protected synchronized boolean doInitialize() {
        if (deviceClient instanceof LynxI2cDeviceSynch) {
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }
        return true;
    }

    public double getRawDegrees() {
        byte[] data = deviceClient.read(Register.RAW_ANGLE_HI.bVal, 2);
        int rawInt = ((data[0] & 0x0F) << 8) | (data[1] & 0xFF);
        return (rawInt / 4096.0) * 360.0;
    }

    public void setZero() {
        this.offsetDegrees = getRawDegrees();
    }

    public double getAngle() {
        double angle = getRawDegrees() - offsetDegrees;
        while (angle < 0) angle += 360.0;
        while (angle >= 360) angle -= 360.0;
        return angle;
    }

    /**
     * AGC ranges from 0 to 255.
     * For a 10mm magnet, aim for a distance where this is roughly 128.
     */
    public int getAgcValue() {
        return deviceClient.read8(Register.AGC.bVal) & 0xFF;
    }

    /**
     * Bit 5: Magnet detected
     * Bit 4: Magnet too weak (too far)
     * Bit 3: Magnet too strong (too close)
     */
    public String getMagnetStatus() {
        int status = deviceClient.read8(Register.STATUS.bVal);
        boolean detected = (status & 0x20) != 0;
        boolean tooFar = (status & 0x10) != 0;
        boolean tooClose = (status & 0x08) != 0;

        if (!detected) return "NOT DETECTED";
        if (tooFar) return "TOO FAR (WEAK)";
        if (tooClose) return "TOO CLOSE (STRONG)";
        return "READY";
    }

    @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
    @Override public String getDeviceName() { return "AS5600 Magnetic Encoder"; }
}