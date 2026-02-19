package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

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

        autoManager.addObjective(AutoManagement.Objective.SHOOT_START);
        autoManager.addObjective(AutoManagement.Objective.SPIKE_MARK_CLOSE);
        autoManager.addObjective(AutoManagement.Objective.SHOOT);
        autoManager.addObjective(AutoManagement.Objective.SPIKE_MARK_MIDDLE);
        autoManager.addObjective(AutoManagement.Objective.SHOOT);
        autoManager.addObjective(AutoManagement.Objective.GATE_RELEASING);
        autoManager.addObjective(AutoManagement.Objective.PARK);

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
            draw();
    }
}