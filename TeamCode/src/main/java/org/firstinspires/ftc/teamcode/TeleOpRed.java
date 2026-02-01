package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotTeleOp;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

public class TeleOpRed extends OpMode {
    private RobotTeleOp robotTeleOp;
    private ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    @Override
    public void init() {
        robotTeleOp = new RobotTeleOp(hardwareMap, Robot.Alliance.RED, PoseStorage.currentPose);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double newTime = getRuntime();
        double loopTime = newTime-previousTime;
        previousTime = newTime;

        robotTeleOp.update(loopTime);
    }

    @Override
    public void stop() {

    }
}
