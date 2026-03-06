package org.firstinspires.ftc.teamcode.TestingTuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Test_Number_Reader")
@Disabled
public class TestNumberReader extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
    }

    @Override
    public void loop() {

        int number = NumberFile.readNumber();

        panelsTelemetry.addLine("NUMBER READER");
        panelsTelemetry.addData("number from file", number);

        telemetry.addData("number from file", number);
    }
}