package org.firstinspires.ftc.teamcode.OpModes;

//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.EIGHTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.ELEVENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.FIFTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.FIRST_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.FOURTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.NINTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.SECOND_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.SEVENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.SIXTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.TENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.THIRD_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigFar.TWELVETH_OBJECTIVE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.lib.AutoManagement;
import org.firstinspires.ftc.teamcode.lib.AutoPoses;


@Autonomous(name = "Auto BLUE Far Configurable", group =  "comp")
public class AutoBlueFarConfigurable extends OpMode {

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private RobotAuto robotAuto;
    private AutoManagement autoManager;

    @Override
    public void init() {

        Pose2D startPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);

        robotAuto = new RobotAuto(hardwareMap, Robot.Alliance.BLUE, startPose);
        AutoPoses.alliance = AutoPoses.Alliance.BLUE;


        autoManager = new AutoManagement(robotAuto, false);

        if(FIRST_OBJECTIVE != null) autoManager.addObjective(FIRST_OBJECTIVE);
        if(SECOND_OBJECTIVE != null) autoManager.addObjective(SECOND_OBJECTIVE);
        if(THIRD_OBJECTIVE != null) autoManager.addObjective(THIRD_OBJECTIVE);
        if(FOURTH_OBJECTIVE != null) autoManager.addObjective(FOURTH_OBJECTIVE);
        if(FIFTH_OBJECTIVE != null) autoManager.addObjective(FIFTH_OBJECTIVE);
        if(SIXTH_OBJECTIVE != null) autoManager.addObjective(SIXTH_OBJECTIVE);
        if(SEVENTH_OBJECTIVE != null) autoManager.addObjective(SEVENTH_OBJECTIVE);
        if(EIGHTH_OBJECTIVE != null) autoManager.addObjective(EIGHTH_OBJECTIVE);
        if(NINTH_OBJECTIVE != null) autoManager.addObjective(NINTH_OBJECTIVE);
        if(TENTH_OBJECTIVE != null) autoManager.addObjective(TENTH_OBJECTIVE);
        if(ELEVENTH_OBJECTIVE != null) autoManager.addObjective(ELEVENTH_OBJECTIVE);
        if(TWELVETH_OBJECTIVE != null) autoManager.addObjective(TWELVETH_OBJECTIVE);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("ready for start");
        telemetry.addLine("");
        robotAuto.update();
        robotAuto.updateTelemetry(panelsTelemetry, telemetry);
    }

    @Override
    public void start() {
        autoManager.start();
    }

    @Override
    public void loop() {
        robotAuto.update();
            // Tasks und Robot-Logik ausführen
            autoManager.update();

            // Telemetrie aktualisieren
            autoManager.updateTelemetry(panelsTelemetry, telemetry);
            //draw();
    }
}