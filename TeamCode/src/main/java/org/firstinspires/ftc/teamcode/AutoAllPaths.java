package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import static org.firstinspires.ftc.teamcode.lib.AutoConfig.EIGHTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.ELEVENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.FIFTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.FIRST_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.FOURTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.NINTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.SECOND_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.SEVENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.SIXTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.TENTH_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.THIRD_OBJECTIVE;
import static org.firstinspires.ftc.teamcode.lib.AutoConfig.TWELVETH_OBJECTIVE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.lib.AutoManagement;


@Autonomous(name = "Auto All Paths", group =  "Comp")
public class AutoAllPaths extends OpMode {
    private RobotAuto robotAuto;
    private AutoManagement autoManager;

    @Override
    public void init() {

        Pose2D startPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.RADIANS,0);

        robotAuto = new RobotAuto(hardwareMap, Robot.Alliance.RED, startPose);

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



        telemetry.addLine("Ready for start");
        telemetry.update();

        autoManager.start();
    }

    @Override
    public void init_loop() {
        autoManager.updateTelemetry();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robotAuto.update();
            // Tasks und Robot-Logik ausführen
            autoManager.update();

            // Telemetrie aktualisieren
            autoManager.updateTelemetry();
            //draw();
    }
}