package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorLED {
    private Servo led;

    public enum Color {
        OFF,
        RED,
        GREEN,
        BLUE,
        ORANGE,
        WHITE,
        PURPLE
    }

    public ColorLED(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");


    }

    public void setColor(Color color) {
        double colorValue = 0;
        switch (color) {
            case OFF:
                colorValue = 0;
                break;
            case RED:
                colorValue = 0.279;
                break;
            case BLUE:
                colorValue = 0.611;
                break;
            case GREEN:
                colorValue = 0.5;
                break;
            case ORANGE:
                colorValue = 0.333;
                break;
            case PURPLE:
                colorValue = 0.722;
                break;
            case WHITE:
                colorValue = 1.0;
                break;
        }
        led.setPosition(colorValue);
    }
}
