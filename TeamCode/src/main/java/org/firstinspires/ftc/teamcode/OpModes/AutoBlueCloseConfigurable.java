package org.firstinspires.ftc.teamcode.OpModes;

//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.EIGHTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.ELEVENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.FIFTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.FIRST_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.FOURTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.NINTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.SECOND_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.SEVENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.SIXTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.TENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.THIRD_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfigClose.TWELVETH_OBJECTIVE;

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


@Autonomous(name = "Auto BLUE Close Configurable", group =  "comp")
public class AutoBlueCloseConfigurable extends OpMode {

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private RobotAuto robotAuto;
    private AutoManagement autoManager;

    @Override
    public void init() {

        Pose2D startPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);

        robotAuto = new RobotAuto(hardwareMap, Robot.Alliance.BLUE, startPose);
        AutoPoses.alliance = AutoPoses.Alliance.BLUE;


        autoManager = new AutoManagement(robotAuto, true);

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