package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotTeleOp;
import org.firstinspires.ftc.teamcode.lib.PoseFile;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

@TeleOp(name = "TeleOp BLUE", group = "comp")
public class TeleOpBlue extends OpMode {
    private RobotTeleOp robotTeleOp;
    //private ElapsedTime runtime = new ElapsedTime();

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        Pose2D savedPose = PoseFile.readPose();

        if(savedPose == null)
            savedPose = PoseStorage.currentPose;

        robotTeleOp = new RobotTeleOp(
                hardwareMap,
                Robot.Alliance.BLUE
        );
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        try {
            robotTeleOp.update(getRuntime());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        robotTeleOp.updateTelemetry(panelsTelemetry, telemetry);
    }

    @Override
    public void stop() {

    }
}
