package org.firstinspires.ftc.teamcode.TestingTuning;

import static org.firstinspires.ftc.teamcode.lib.Drawing.drawDebug;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DrivetrainTeleOp;
import org.firstinspires.ftc.teamcode.lib.Controller;

@TeleOp(name = "Localizer Test", group = "Test")
public class LocalizerTest extends OpMode {
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public DrivetrainTeleOp drivetrainTeleOp;


    @Override
    public void init() {
        drivetrainTeleOp = new DrivetrainTeleOp(hardwareMap,new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
    }

    @Override
    public void loop() {
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        try {
            drivetrainTeleOp.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, drivetrainTeleOp.getPose());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        Follower followerFused = drivetrainTeleOp.getFollower();

        drawDebug(drivetrainTeleOp.getFollower());
        panelsTelemetry.update();
    }
}
