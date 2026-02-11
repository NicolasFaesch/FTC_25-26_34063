package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotTeleOp;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

import java.util.Locale;
import java.util.Objects;

import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "Shooter PIDF Tuning", group = "Tuning")
@Configurable
public class ShooterPIDFTuning extends OpMode {
    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();

    private RobotTeleOp robotTeleOp;
    private ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0;

    @Override
    public void init() {
        robotTeleOp = new RobotTeleOp(hardwareMap, Robot.Alliance.RED, PoseStorage.currentPose
                , gamepad1, gamepad2);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(gamepad1.dpadRightWasPressed())
            robotTeleOp.shooter.setShooterPIDFGains(kP, kI, kD, kV);
        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        double newTime = getRuntime();
        double loopTime = newTime-previousTime;
        previousTime = newTime;

        try {
            robotTeleOp.update(loopTime);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        Pose2D botpose = robotTeleOp.drivetrainTeleOp.getPose();
        String position = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f", botpose.getX(DistanceUnit.INCH), botpose.getY(DistanceUnit.INCH), botpose.getHeading(AngleUnit.DEGREES));
        String velocity = String.format(Locale.US,"XVel: %.3f, YVel: %.3f, AVel: %.3f", robotTeleOp.drivetrainTeleOp.getVelocityX(), robotTeleOp.drivetrainTeleOp.getVelocityY(), robotTeleOp.drivetrainTeleOp.getAngularVelocity());

        telemetry.addData("Loop Time [ms]", loopTime*1000);

        telemetry.addLine("");

        telemetry.addLine("STATES:");
        telemetry.addData("Robot State", robotTeleOp.getState());
        telemetry.addData("Intake State", robotTeleOp.intake.getState());
        telemetry.addData("Transfer State", robotTeleOp.transfer.getState());
        telemetry.addData("Shooter State", robotTeleOp.shooter.getState());

        telemetry.addLine("");

        // Robot pose telemetry
        if(!robotTeleOp.isUsingLimelight()) {
            telemetry.addData("Pose (Odometry)", position);
        } else {
            telemetry.addData("Pose (Limelight)", position);
        }
        telemetry.addData("Distance", robotTeleOp.drivetrainTeleOp.getDistance());


        telemetry.addLine("");

        // Shooter telemetry
        if(robotTeleOp.shooter.getManualOverride()) {
            telemetry.addLine("Shooter: MANUAL OVERRIDE");
            telemetry.addData("Hood Position (manual)", robotTeleOp.shooter.getHoodPositionManual());
            telemetry.addData("Shooter Target Velocity (manual)", robotTeleOp.shooter.getShooterTargetVelocityManual());
            panelsTelemetry.addData("Shooter Target Velocity", robotTeleOp.shooter.getShooterTargetVelocityManual());
        } else {
            telemetry.addLine("Shooter:");
            telemetry.addData("Hood Position", robotTeleOp.shooter.getHoodPosition());
            telemetry.addData("Shooter Target Velocity", robotTeleOp.shooter.getShooterTargetVelocity());
            panelsTelemetry.addData("Shooter Target Velocity", robotTeleOp.shooter.getShooterTargetVelocity());
        }
        telemetry.addData("Shooter Velocity", robotTeleOp.shooter.getShooterVelocity());
        panelsTelemetry.addData("Shooter Velocity", robotTeleOp.shooter.getShooterVelocity());


        telemetry.update();
        panelsTelemetry.update();
    }

    @Override
    public void stop() {

    }
}
