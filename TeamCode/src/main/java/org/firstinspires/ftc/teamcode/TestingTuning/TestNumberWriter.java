package org.firstinspires.ftc.teamcode.TestingTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Configurable
@Autonomous(name = "Test_Number_Writer")
@Disabled
public class TestNumberWriter extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    int number = 33;

    boolean upPressed = false;
    boolean downPressed = false;

    @Override
    public void init() {
    }

    @Override
    public void loop() {

        // Zahl ändern
        if(gamepad1.dpad_up && !upPressed){
            number++;
        }

        if(gamepad1.dpad_down && !downPressed){
            number--;
        }

        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;

        // In Datei speichern
        NumberFile.writeNumber(number);

        // Panel Telemetry
        panelsTelemetry.addLine("NUMBER WRITER");
        panelsTelemetry.addData("current number", number);

        telemetry.addData("current number", number);
    }
}